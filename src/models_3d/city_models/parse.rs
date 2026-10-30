//! Readers for open city-model files: CityJSON (3D BAG and others), CityGML (German LoD2,
//! NYC, Helsinki, ...) and OBJ. Each yields buildings as triangles in the file's own CRS.

use super::crs::{parse_epsg, urn_is_lat_first, Crs};
use super::triangulate::triangulate_ring;
use serde_json::Value;
use std::collections::HashSet;
use std::path::{Path, PathBuf};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Surface {
    Roof,
    Wall,
    Ground,
    Other,
}

#[derive(Clone, Copy, Debug)]
pub struct Tri {
    pub v: [[f64; 3]; 3],
    pub surface: Surface,
}

#[derive(Clone, Debug)]
pub struct SrcBuilding {
    pub id: String,
    pub tris: Vec<Tri>,
}

/// Buildings of one file together with the CRS their coordinates are in.
#[derive(Debug)]
pub struct Loaded {
    pub crs: Crs,
    pub buildings: Vec<SrcBuilding>,
}

/// Options that cannot be read from the files themselves.
#[derive(Clone, Copy, Default)]
pub struct LoadOptions {
    /// Forced CRS, required for OBJ and for files that do not name one.
    pub crs: Option<Crs>,
    /// OBJ files with Y up instead of Z up.
    pub y_up: bool,
}

/// Loads a file, or every supported file under a directory.
pub fn load_path(path: &Path, opts: LoadOptions) -> Result<Vec<Loaded>, String> {
    let mut files: Vec<PathBuf> = Vec::new();
    collect_files(path, &mut files)?;
    if files.is_empty() {
        return Err(format!(
            "no .json/.jsonl/.gml/.xml/.obj city model files found in {}",
            path.display()
        ));
    }
    files.sort();
    let mut out = Vec::new();
    for file in files {
        match load_file(&file, opts) {
            Ok(mut loaded) => out.append(&mut loaded),
            Err(e) => eprintln!("Warning: city model {}: {e}", file.display()),
        }
    }
    Ok(out)
}

fn collect_files(path: &Path, out: &mut Vec<PathBuf>) -> Result<(), String> {
    if path.is_dir() {
        let entries = std::fs::read_dir(path).map_err(|e| format!("{}: {e}", path.display()))?;
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_dir() || supported_extension(&p).is_some() {
                collect_files(&p, out)?;
            }
        }
        Ok(())
    } else if path.is_file() {
        out.push(path.to_path_buf());
        Ok(())
    } else {
        Err(format!("{} does not exist", path.display()))
    }
}

fn supported_extension(path: &Path) -> Option<String> {
    let ext = path.extension()?.to_str()?.to_ascii_lowercase();
    matches!(ext.as_str(), "json" | "jsonl" | "gml" | "xml" | "obj").then_some(ext)
}

fn load_file(path: &Path, opts: LoadOptions) -> Result<Vec<Loaded>, String> {
    let ext = supported_extension(path).ok_or("unsupported file type")?;
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    let text = String::from_utf8_lossy(&bytes);
    match ext.as_str() {
        "json" | "jsonl" => parse_cityjson(&text, opts),
        "gml" | "xml" => parse_citygml(&text, opts),
        _ => parse_obj(&text, opts),
    }
}

fn unsupported_crs(text: &str) -> String {
    format!(
        "coordinate system \"{text}\" is not supported (supported: WGS84, Web Mercator, UTM/ETRS89 \
         zones, Dutch RD 28992/7415 and NY Long Island 2263). Pass --city-model-crs with one of those."
    )
}

// ---------------------------------------------------------------- CityJSON

fn lod_of(geometry: &Value) -> f64 {
    match geometry.get("lod") {
        Some(Value::String(s)) => s.trim().parse().unwrap_or(0.0),
        Some(Value::Number(n)) => n.as_f64().unwrap_or(0.0),
        _ => 0.0,
    }
}

fn surface_of(kind: &str) -> Surface {
    match kind {
        "RoofSurface" => Surface::Roof,
        "WallSurface" => Surface::Wall,
        "GroundSurface" => Surface::Ground,
        _ => Surface::Other,
    }
}

struct JsonTransform {
    scale: [f64; 3],
    translate: [f64; 3],
}

impl JsonTransform {
    fn from(doc: &Value) -> Self {
        let triple = |key: &str, default: f64| -> [f64; 3] {
            doc.get("transform")
                .and_then(|t| t.get(key))
                .and_then(Value::as_array)
                .map(|a| {
                    let g = |i: usize| a.get(i).and_then(Value::as_f64).unwrap_or(default);
                    [g(0), g(1), g(2)]
                })
                .unwrap_or([default; 3])
        };
        Self {
            scale: triple("scale", 1.0),
            translate: triple("translate", 0.0),
        }
    }

    fn vertices(&self, doc: &Value) -> Vec<[f64; 3]> {
        doc.get("vertices")
            .and_then(Value::as_array)
            .map(|vs| {
                vs.iter()
                    .map(|v| {
                        let g = |i: usize| v.get(i).and_then(Value::as_f64).unwrap_or(0.0);
                        [
                            g(0) * self.scale[0] + self.translate[0],
                            g(1) * self.scale[1] + self.translate[1],
                            g(2) * self.scale[2] + self.translate[2],
                        ]
                    })
                    .collect()
            })
            .unwrap_or_default()
    }
}

fn json_crs(doc: &Value) -> Option<String> {
    doc.get("metadata")
        .and_then(|m| m.get("referenceSystem"))
        .and_then(Value::as_str)
        .map(str::to_string)
}

fn resolve_crs(named: Option<&str>, opts: LoadOptions) -> Result<Crs, String> {
    if let Some(forced) = opts.crs {
        return Ok(forced);
    }
    let text = named.ok_or("the file names no coordinate system; pass --city-model-crs")?;
    parse_epsg(text)
        .and_then(Crs::from_epsg)
        .or_else(|| utm_zone_in_name(text))
        .ok_or_else(|| unsupported_crs(text))
}

/// German AdV names such as `urn:adv:crs:ETRS89_UTM32*DE_DHHN2016_NH` carry the UTM zone only.
fn utm_zone_in_name(text: &str) -> Option<Crs> {
    let upper = text.to_ascii_uppercase();
    let after = &upper[upper.find("UTM")? + 3..];
    let digits: String = after.chars().take_while(char::is_ascii_digit).collect();
    let zone: u8 = digits.parse().ok()?;
    (1..=60).contains(&zone).then_some(Crs::Utm { zone, north: true })
}

fn parse_cityjson(text: &str, opts: LoadOptions) -> Result<Vec<Loaded>, String> {
    // A whole CityJSON document, or a CityJSONSeq whose first line carries the header.
    if let Ok(doc) = serde_json::from_str::<Value>(text) {
        if doc.get("type").and_then(Value::as_str) == Some("CityJSON") {
            let crs = resolve_crs(json_crs(&doc).as_deref(), opts)?;
            let verts = JsonTransform::from(&doc).vertices(&doc);
            let buildings = objects_to_buildings(&doc, &verts);
            return Ok(vec![Loaded { crs, buildings }]);
        }
    }
    let mut lines = text.lines().filter(|l| !l.trim().is_empty());
    let header: Value = serde_json::from_str(lines.next().ok_or("empty file")?)
        .map_err(|e| format!("not CityJSON: {e}"))?;
    if header.get("type").and_then(Value::as_str) != Some("CityJSON") {
        return Err("not a CityJSON document or CityJSONSeq stream".to_string());
    }
    let crs = resolve_crs(json_crs(&header).as_deref(), opts)?;
    let transform = JsonTransform::from(&header);
    let mut buildings = Vec::new();
    for line in lines {
        let Ok(feature) = serde_json::from_str::<Value>(line) else {
            continue;
        };
        let verts = transform.vertices(&feature);
        buildings.extend(objects_to_buildings(&feature, &verts));
    }
    Ok(vec![Loaded { crs, buildings }])
}

fn has_solid_geometry(object: &Value) -> bool {
    object
        .get("geometry")
        .and_then(Value::as_array)
        .is_some_and(|g| g.iter().any(|g| lod_of(g) >= 1.0))
}

fn objects_to_buildings(doc: &Value, verts: &[[f64; 3]]) -> Vec<SrcBuilding> {
    let Some(objects) = doc.get("CityObjects").and_then(Value::as_object) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for (id, object) in objects {
        let kind = object.get("type").and_then(Value::as_str).unwrap_or("");
        if !matches!(kind, "Building" | "BuildingPart" | "BuildingInstallation") {
            continue;
        }
        // A building whose parts carry the geometry would otherwise be drawn twice.
        let parts_carry_geometry = object
            .get("children")
            .and_then(Value::as_array)
            .is_some_and(|c| {
                c.iter()
                    .filter_map(Value::as_str)
                    .filter_map(|cid| objects.get(cid))
                    .any(has_solid_geometry)
            });
        if parts_carry_geometry {
            continue;
        }
        let Some(geometries) = object.get("geometry").and_then(Value::as_array) else {
            continue;
        };
        let best = geometries
            .iter()
            .filter(|g| lod_of(g) >= 1.0)
            .max_by(|a, b| lod_of(a).total_cmp(&lod_of(b)));
        let Some(geometry) = best else { continue };
        let tris = geometry_triangles(geometry, verts);
        if !tris.is_empty() {
            out.push(SrcBuilding {
                id: id.clone(),
                tris,
            });
        }
    }
    out
}

/// Surfaces (each a list of rings) of a geometry, in boundary order.
fn collect_surfaces<'a>(boundaries: &'a Value, depth: usize, out: &mut Vec<&'a Value>) {
    if depth == 0 {
        if let Some(surfaces) = boundaries.as_array() {
            out.extend(surfaces.iter());
        }
        return;
    }
    if let Some(items) = boundaries.as_array() {
        for item in items {
            collect_surfaces(item, depth - 1, out);
        }
    }
}

fn flatten_values(values: &Value, out: &mut Vec<Option<usize>>) {
    match values {
        Value::Array(items) => items.iter().for_each(|v| flatten_values(v, out)),
        Value::Number(n) => out.push(n.as_u64().map(|v| v as usize)),
        _ => out.push(None),
    }
}

fn geometry_triangles(geometry: &Value, verts: &[[f64; 3]]) -> Vec<Tri> {
    let depth = match geometry.get("type").and_then(Value::as_str) {
        Some("MultiSurface") | Some("CompositeSurface") => 0,
        Some("Solid") => 1,
        Some("MultiSolid") | Some("CompositeSolid") => 2,
        _ => return Vec::new(),
    };
    let Some(boundaries) = geometry.get("boundaries") else {
        return Vec::new();
    };
    let mut surfaces = Vec::new();
    collect_surfaces(boundaries, depth, &mut surfaces);

    let semantic_types: Vec<Surface> = geometry
        .get("semantics")
        .and_then(|s| s.get("surfaces"))
        .and_then(Value::as_array)
        .map(|a| {
            a.iter()
                .map(|s| surface_of(s.get("type").and_then(Value::as_str).unwrap_or("")))
                .collect()
        })
        .unwrap_or_default();
    let mut values = Vec::new();
    if let Some(v) = geometry.get("semantics").and_then(|s| s.get("values")) {
        flatten_values(v, &mut values);
    }

    let mut tris = Vec::new();
    for (i, surface) in surfaces.iter().enumerate() {
        let class = values
            .get(i)
            .copied()
            .flatten()
            .and_then(|k| semantic_types.get(k).copied())
            .unwrap_or(Surface::Other);
        // Only the outer ring; openings in a wall are filled in.
        let Some(ring) = surface
            .as_array()
            .and_then(|rings| rings.first())
            .and_then(Value::as_array)
        else {
            continue;
        };
        let pts: Vec<[f64; 3]> = ring
            .iter()
            .filter_map(|v| v.as_u64().and_then(|i| verts.get(i as usize).copied()))
            .collect();
        for t in triangulate_ring(&pts) {
            tris.push(Tri {
                v: [pts[t[0]], pts[t[1]], pts[t[2]]],
                surface: class,
            });
        }
    }
    tris
}

// ---------------------------------------------------------------- CityGML

struct GmlBuilding {
    id: String,
    /// (lod, polygon id, class, ring)
    polys: Vec<(u8, String, Surface, Vec<[f64; 3]>)>,
}

fn lod_digit(name: &str) -> Option<u8> {
    name.strip_prefix("lod")
        .and_then(|r| r.chars().next())
        .and_then(|c| c.to_digit(10))
        .map(|d| d as u8)
}

fn gml_attr(e: &quick_xml::events::BytesStart, key: &[u8]) -> Option<String> {
    for attr in e.attributes().flatten() {
        // `gml:id` and `id` both count; the prefix is irrelevant.
        let name = attr.key.into_inner();
        let local = name.rsplit(|b| *b == b':').next().unwrap_or(name);
        if local == key {
            return attr
                .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                .ok()
                .map(|v| v.into_owned());
        }
    }
    None
}

fn parse_numbers(text: &str, dim: usize, out: &mut Vec<[f64; 3]>) {
    let nums: Vec<f64> = text
        .split(|c: char| c.is_whitespace() || c == ',')
        .filter(|s| !s.is_empty())
        .filter_map(|s| s.parse().ok())
        .collect();
    if dim < 3 {
        return;
    }
    for c in nums.chunks_exact(dim) {
        out.push([c[0], c[1], c[2]]);
    }
}

fn parse_citygml(text: &str, opts: LoadOptions) -> Result<Vec<Loaded>, String> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let mut xml = Reader::from_str(text);
    let mut stack: Vec<String> = Vec::new();
    let mut buildings: Vec<GmlBuilding> = Vec::new();
    let mut open: Vec<GmlBuilding> = Vec::new();
    let mut srs_name: Option<String> = None;

    // Polygon being read.
    let mut poly_id = String::new();
    let mut in_polygon = false;
    let mut interior = false;
    let mut ring: Vec<[f64; 3]> = Vec::new();
    let mut ring_dim = 3usize;
    let mut want_text = false;
    let mut exterior: Option<Vec<[f64; 3]>> = None;

    loop {
        match xml.read_event().map_err(|e| e.to_string())? {
            Event::Start(e) => {
                let name = String::from_utf8_lossy(e.local_name().as_ref()).into_owned();
                if srs_name.is_none() {
                    srs_name = gml_attr(&e, b"srsName");
                }
                match name.as_str() {
                    "Building" | "BuildingPart" => open.push(GmlBuilding {
                        id: gml_attr(&e, b"id").unwrap_or_default(),
                        polys: Vec::new(),
                    }),
                    "Polygon" if !open.is_empty() => {
                        in_polygon = true;
                        poly_id = gml_attr(&e, b"id").unwrap_or_default();
                        exterior = None;
                    }
                    "interior" if in_polygon => interior = true,
                    "LinearRing" if in_polygon => ring.clear(),
                    "posList" | "pos" | "coordinates" if in_polygon => {
                        want_text = true;
                        ring_dim = gml_attr(&e, b"srsDimension")
                            .and_then(|d| d.parse().ok())
                            .unwrap_or(3);
                        if name == "coordinates" {
                            ring_dim = 3;
                        }
                    }
                    _ => {}
                }
                stack.push(name);
            }
            Event::Empty(e) => {
                if srs_name.is_none() {
                    srs_name = gml_attr(&e, b"srsName");
                }
            }
            Event::Text(t) if want_text => {
                let s = t.xml10_content().map_err(|e| e.to_string())?;
                parse_numbers(&s, ring_dim, &mut ring);
            }
            Event::End(e) => {
                let name = String::from_utf8_lossy(e.local_name().as_ref()).into_owned();
                stack.pop();
                match name.as_str() {
                    "posList" | "pos" | "coordinates" => want_text = false,
                    "interior" => interior = false,
                    "LinearRing" if in_polygon && !interior && exterior.is_none() => {
                        exterior = Some(std::mem::take(&mut ring));
                    }
                    "Polygon" if in_polygon => {
                        in_polygon = false;
                        if let (Some(b), Some(r)) = (open.last_mut(), exterior.take()) {
                            let lod = stack.iter().rev().find_map(|n| lod_digit(n)).unwrap_or(2);
                            let class = stack
                                .iter()
                                .rev()
                                .find_map(|n| match n.as_str() {
                                    "RoofSurface" => Some(Surface::Roof),
                                    "WallSurface" => Some(Surface::Wall),
                                    "GroundSurface" => Some(Surface::Ground),
                                    _ => None,
                                })
                                .unwrap_or(Surface::Other);
                            b.polys.push((lod, std::mem::take(&mut poly_id), class, r));
                        }
                    }
                    "Building" | "BuildingPart" => {
                        if let Some(b) = open.pop() {
                            buildings.push(b);
                        }
                    }
                    _ => {}
                }
            }
            Event::Eof => break,
            _ => {}
        }
    }

    let crs = resolve_crs(srs_name.as_deref(), opts)?;
    // GML geographic URNs put latitude first.
    let crs = match (crs, srs_name.as_deref()) {
        (Crs::Geographic { .. }, Some(n)) if urn_is_lat_first(n, &crs) => {
            Crs::Geographic { lat_first: true }
        }
        _ => crs,
    };

    let mut out = Vec::new();
    for b in buildings {
        // The richest model the building has; LoD0 is only a footprint.
        let Some(best) = b.polys.iter().map(|p| p.0).filter(|l| *l >= 1).max() else {
            continue;
        };
        let mut seen: HashSet<&str> = HashSet::new();
        let mut tris = Vec::new();
        for (lod, id, class, ring) in &b.polys {
            if *lod != best || (!id.is_empty() && !seen.insert(id.as_str())) {
                continue;
            }
            for t in triangulate_ring(ring) {
                tris.push(Tri {
                    v: [ring[t[0]], ring[t[1]], ring[t[2]]],
                    surface: *class,
                });
            }
        }
        if !tris.is_empty() {
            out.push(SrcBuilding { id: b.id, tris });
        }
    }
    Ok(vec![Loaded {
        crs,
        buildings: out,
    }])
}

// ---------------------------------------------------------------- OBJ

fn parse_obj(text: &str, opts: LoadOptions) -> Result<Vec<Loaded>, String> {
    let crs = opts.crs.ok_or(
        "OBJ files carry no coordinate system; pass --city-model-crs (for example EPSG:25832)",
    )?;
    // Real-world axes: x east, y north, z up.
    let map = |x: f64, y: f64, z: f64| {
        if opts.y_up {
            [x, -z, y]
        } else {
            [x, y, z]
        }
    };
    let mut verts: Vec<[f64; 3]> = Vec::new();
    let mut groups: Vec<SrcBuilding> = vec![SrcBuilding {
        id: "obj".to_string(),
        tris: Vec::new(),
    }];

    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("v ") {
            let n: Vec<f64> = rest
                .split_whitespace()
                .take(3)
                .filter_map(|s| s.parse().ok())
                .collect();
            if n.len() == 3 {
                verts.push(map(n[0], n[1], n[2]));
            }
        } else if let Some(rest) = line.strip_prefix("o ").or_else(|| line.strip_prefix("g ")) {
            if groups.last().is_some_and(|g| !g.tris.is_empty()) {
                groups.push(SrcBuilding {
                    id: rest.trim().to_string(),
                    tris: Vec::new(),
                });
            } else if let Some(g) = groups.last_mut() {
                g.id = rest.trim().to_string();
            }
        } else if let Some(rest) = line.strip_prefix("f ") {
            let ring: Vec<[f64; 3]> = rest
                .split_whitespace()
                .filter_map(|tok| {
                    let i: i64 = tok.split('/').next()?.parse().ok()?;
                    let idx = if i < 0 { verts.len() as i64 + i } else { i - 1 };
                    verts.get(usize::try_from(idx).ok()?).copied()
                })
                .collect();
            for t in triangulate_ring(&ring) {
                let v = [ring[t[0]], ring[t[1]], ring[t[2]]];
                let n = triangle_normal(&v);
                let surface = if n[2] > 0.7 {
                    Surface::Roof
                } else if n[2] < -0.7 {
                    Surface::Ground
                } else {
                    Surface::Wall
                };
                if let Some(g) = groups.last_mut() {
                    g.tris.push(Tri { v, surface });
                }
            }
        }
    }
    groups.retain(|g| !g.tris.is_empty());
    Ok(vec![Loaded {
        crs,
        buildings: groups,
    }])
}

fn triangle_normal(v: &[[f64; 3]; 3]) -> [f64; 3] {
    let a = [v[1][0] - v[0][0], v[1][1] - v[0][1], v[1][2] - v[0][2]];
    let b = [v[2][0] - v[0][0], v[2][1] - v[0][1], v[2][2] - v[0][2]];
    let n = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        [n[0] / len, n[1] / len, n[2] / len]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CITYJSON: &str = r#"{
      "type":"CityJSON","version":"2.0",
      "metadata":{"referenceSystem":"https://www.opengis.net/def/crs/EPSG/0/7415"},
      "transform":{"scale":[1,1,1],"translate":[0,0,0]},
      "CityObjects":{
        "b1":{"type":"Building","children":["b1-0"]},
        "b1-0":{"type":"BuildingPart","parents":["b1"],"geometry":[
          {"type":"Solid","lod":"1.2","boundaries":[[[[0,1,2,3]]]]},
          {"type":"Solid","lod":"2.2","boundaries":[[
            [[0,1,2,3]], [[4,5,6,7]], [[0,1,5,4]], [[1,2,6,5]], [[2,3,7,6]], [[3,0,4,7]]
          ]],"semantics":{"surfaces":[{"type":"GroundSurface"},{"type":"RoofSurface"},{"type":"WallSurface"}],
            "values":[[0,1,2,2,2,2]]}}
        ]}
      },
      "vertices":[[0,0,0],[10,0,0],[10,10,0],[0,10,0],[0,0,6],[10,0,6],[10,10,6],[0,10,6]]
    }"#;

    #[test]
    fn cityjson_picks_highest_lod_and_semantics() {
        let loaded = parse_cityjson(CITYJSON, LoadOptions::default()).unwrap();
        assert_eq!(loaded.len(), 1);
        assert_eq!(loaded[0].crs, Crs::AmersfoortRd);
        let b = &loaded[0].buildings;
        assert_eq!(
            b.len(),
            1,
            "the parent Building is skipped, its part is kept"
        );
        assert_eq!(b[0].id, "b1-0");
        assert_eq!(b[0].tris.len(), 12);
        assert_eq!(
            b[0].tris
                .iter()
                .filter(|t| t.surface == Surface::Roof)
                .count(),
            2
        );
        assert_eq!(
            b[0].tris
                .iter()
                .filter(|t| t.surface == Surface::Wall)
                .count(),
            8
        );
    }

    const CITYGML: &str = r#"<?xml version="1.0"?>
    <core:CityModel xmlns:core="http://www.opengis.net/citygml/2.0"
        xmlns:bldg="http://www.opengis.net/citygml/building/2.0"
        xmlns:gml="http://www.opengis.net/gml">
      <gml:boundedBy><gml:Envelope srsName="urn:adv:crs:ETRS89_UTM32" srsDimension="3"/></gml:boundedBy>
      <core:cityObjectMember>
        <bldg:Building gml:id="B1">
          <bldg:boundedBy><bldg:RoofSurface><bldg:lod2MultiSurface><gml:MultiSurface>
            <gml:surfaceMember><gml:Polygon gml:id="P1"><gml:exterior><gml:LinearRing>
              <gml:posList srsDimension="3">0 0 6 10 0 6 10 10 6 0 10 6 0 0 6</gml:posList>
            </gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
          </gml:MultiSurface></bldg:lod2MultiSurface></bldg:RoofSurface></bldg:boundedBy>
          <bldg:boundedBy><bldg:WallSurface><bldg:lod2MultiSurface><gml:MultiSurface>
            <gml:surfaceMember><gml:Polygon gml:id="P2"><gml:exterior><gml:LinearRing>
              <gml:posList srsDimension="3">0 0 0 10 0 0 10 0 6 0 0 6 0 0 0</gml:posList>
            </gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
          </gml:MultiSurface></bldg:lod2MultiSurface></bldg:WallSurface></bldg:boundedBy>
        </bldg:Building>
      </core:cityObjectMember>
    </core:CityModel>"#;

    #[test]
    fn citygml_reads_polygons_and_classes() {
        let opts = LoadOptions {
            crs: Crs::from_epsg(25832),
            y_up: false,
        };
        let loaded = parse_citygml(CITYGML, opts).unwrap();
        let b = &loaded[0].buildings;
        assert_eq!(b.len(), 1);
        assert_eq!(b[0].tris.len(), 4);
        assert_eq!(
            b[0].tris
                .iter()
                .filter(|t| t.surface == Surface::Roof)
                .count(),
            2
        );
    }

    #[test]
    fn citygml_without_a_known_crs_asks_for_one() {
        let err = parse_citygml(CITYGML, LoadOptions::default()).unwrap_err();
        assert!(err.contains("--city-model-crs"), "{err}");
    }

    #[test]
    fn obj_needs_crs_and_classifies_by_normal() {
        let obj =
            "o house\nv 0 0 0\nv 4 0 0\nv 4 4 0\nv 0 4 0\nv 0 0 3\nv 4 0 3\nv 4 4 3\nv 0 4 3\n\
                   f 5 6 7 8\nf 1 2 6 5\n";
        assert!(parse_obj(obj, LoadOptions::default()).is_err());
        let loaded = parse_obj(
            obj,
            LoadOptions {
                crs: Crs::from_epsg(25832),
                y_up: false,
            },
        )
        .unwrap();
        let b = &loaded[0].buildings[0];
        assert_eq!(b.id, "house");
        assert!(b.tris.iter().any(|t| t.surface == Surface::Roof));
        assert!(b.tris.iter().any(|t| t.surface == Surface::Wall));
    }
}
