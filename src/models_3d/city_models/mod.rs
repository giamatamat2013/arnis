//! Real 3D city models (CityJSON, CityGML, OBJ) placed in place of the OSM buildings under them.
//!
//! The files come from an open-data portal (3D BAG, German LoD2, NYC 3D Model, ...). Every
//! vertex is projected to the world, so walls, roofs and heights follow the real building.

pub(crate) mod crs;
pub(crate) mod parse;
pub(crate) mod triangulate;

use crate::args::Args;
use crate::block_definitions::{
    Block, BRICK, BROWN_TERRACOTTA, DEEPSLATE_TILES, GLASS, GRAY_CONCRETE, LIGHT_GRAY_CONCRETE,
    POLISHED_ANDESITE, RED_TERRACOTTA, SMOOTH_STONE, STONE_BRICKS,
};
use crate::block_palette::closest_block;
use crate::colors::color_text_to_rgb_tuple;
use crate::coordinate_system::cartesian::XZBBox;
use crate::coordinate_system::geographic::{LLBBox, LLPoint};
use crate::osm_parser::ProcessedElement;
use crate::world_editor::WorldEditor;
use colored::Colorize;
use dda_voxelize::DdaVoxelizer;
use parse::{LoadOptions, Surface};
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};

/// One building ready to be voxelized: triangles in world blocks (x, z) with y in blocks
/// above the building's lowest point.
struct Placement {
    #[allow(dead_code)]
    id: String,
    tris: Vec<([[f32; 3]; 3], Surface)>,
    min_x: i32,
    max_x: i32,
    min_z: i32,
    max_z: i32,
    wall: Block,
    roof: Block,
    flat_roof: Block,
}

impl Placement {
    fn bbox_contains(&self, x: i32, z: i32) -> bool {
        x >= self.min_x && x <= self.max_x && z >= self.min_z && z <= self.max_z
    }

    /// True when (x, z) lies under any roof or floor triangle seen from above.
    fn covers(&self, x: f32, z: f32) -> bool {
        self.tris
            .iter()
            .filter(|(_, s)| matches!(s, Surface::Roof | Surface::Ground | Surface::Other))
            .any(|(t, _)| point_in_triangle_xz(x, z, t))
    }
}

pub struct PrescanResult {
    pub suppressed_ids: HashSet<(&'static str, u64)>,
    placements: Vec<Placement>,
}

impl PrescanResult {
    fn empty() -> Self {
        Self {
            suppressed_ids: HashSet::new(),
            placements: Vec::new(),
        }
    }

    pub fn placement_count(&self) -> usize {
        self.placements.len()
    }

    pub fn placement_regions(&self, scale: f64) -> Vec<Vec<(i32, i32)>> {
        let _ = scale;
        self.placements
            .iter()
            .map(|p| {
                let cx = (p.min_x + p.max_x) / 2;
                let cz = (p.min_z + p.max_z) / 2;
                let r = ((p.max_x - p.min_x).max(p.max_z - p.min_z) / 2 + 4).max(8);
                crate::models_3d::region_keys_around(cx, cz, r)
            })
            .collect()
    }
}

/// World position of a longitude/latitude, as fractional blocks.
struct Projector {
    transformer: crate::coordinate_system::transformation::CoordTransformer,
    pre_rotation_bbox: XZBBox,
    rotation: f64,
}

impl Projector {
    fn new(llbbox: &LLBBox, args: &Args) -> Option<Self> {
        let (transformer, pre_rotation_bbox) = crate::projection::ProjectionSpec::from_args(args)
            .transformer(llbbox)
            .ok()?;
        Some(Self {
            transformer,
            pre_rotation_bbox,
            rotation: args.rotation,
        })
    }

    /// Voxel centres sit on integers while block `i` spans `[i, i + 1)`, hence the half block.
    fn world_xz(&self, lon: f64, lat: f64) -> Option<(f32, f32)> {
        let (mut x, mut z) = self
            .transformer
            .transform_point_f64(LLPoint::new(lat, lon).ok()?);
        if self.rotation.abs() >= f64::EPSILON {
            let rad = (-self.rotation).to_radians();
            let b = &self.pre_rotation_bbox;
            let cx = (b.min_x() + b.max_x()) as f64 / 2.0;
            let cz = (b.min_z() + b.max_z()) as f64 / 2.0;
            let (dx, dz) = (x - cx, z - cz);
            x = dx * rad.cos() + dz * rad.sin() + cx;
            z = -dx * rad.sin() + dz * rad.cos() + cz;
        }
        Some(((x - 0.5) as f32, (z - 0.5) as f32))
    }
}

fn crs_option(args: &Args) -> Result<Option<crs::Crs>, String> {
    args.city_model_crs
        .as_deref()
        .map(crs::crs_from_arg)
        .transpose()
}

pub fn prescan(
    elements: &[ProcessedElement],
    already_suppressed: &HashSet<(&'static str, u64)>,
    llbbox: LLBBox,
    xzbbox: &XZBBox,
    args: &Args,
) -> PrescanResult {
    let Some(path) = args.city_model.as_ref() else {
        return PrescanResult::empty();
    };
    let forced = match crs_option(args) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{} {e}", "Warning:".yellow().bold());
            return PrescanResult::empty();
        }
    };
    let Some(projector) = Projector::new(&llbbox, args) else {
        eprintln!(
            "{} city model: could not set up the world projection",
            "Warning:".yellow().bold()
        );
        return PrescanResult::empty();
    };

    println!(
        "{} Reading city model {}...",
        "  [+]".bold(),
        path.display()
    );
    let loaded = match parse::load_path(
        path,
        LoadOptions {
            crs: forced,
            y_up: args.city_model_yup,
        },
    ) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("{} city model: {e}", "Warning:".yellow().bold());
            return PrescanResult::empty();
        }
    };

    // Margin around the area, so a building that straddles the edge still counts.
    let margin = 0.01;
    let (min_lat, max_lat) = (llbbox.min().lat() - margin, llbbox.max().lat() + margin);
    let (min_lon, max_lon) = (llbbox.min().lng() - margin, llbbox.max().lng() + margin);
    let in_area = |lon: f64, lat: f64| {
        (min_lon..=max_lon).contains(&lon) && (min_lat..=max_lat).contains(&lat)
    };

    let scale = args.scale as f32;
    let mut placements: Vec<Placement> = Vec::new();
    for file in &loaded {
        let unit = file.crs.unit_m();
        let candidates: Vec<&parse::SrcBuilding> = file
            .buildings
            .iter()
            .filter(|b| {
                b.tris.first().is_some_and(|t| {
                    let (lon, lat) = file.crs.to_lonlat(t.v[0][0], t.v[0][1]);
                    in_area(lon, lat)
                })
            })
            .collect();
        let built: Vec<Option<Placement>> = candidates
            .par_iter()
            .map(|b| build_placement(b, file.crs, unit, &projector, scale, xzbbox))
            .collect();
        placements.extend(built.into_iter().flatten());
    }

    if placements.is_empty() {
        println!("  City model: no buildings inside the generated area");
        return PrescanResult::empty();
    }

    let suppressed = suppress_osm_buildings(elements, already_suppressed, &mut placements);
    println!(
        "  City model: {} buildings replace {} OpenStreetMap buildings",
        placements.len().to_string().bright_white().bold(),
        suppressed.len()
    );
    PrescanResult {
        suppressed_ids: suppressed,
        placements,
    }
}

fn build_placement(
    b: &parse::SrcBuilding,
    crs: crs::Crs,
    unit: f64,
    projector: &Projector,
    scale: f32,
    xzbbox: &XZBBox,
) -> Option<Placement> {
    let z_min = b
        .tris
        .iter()
        .flat_map(|t| t.v.iter().map(|v| v[2]))
        .fold(f64::INFINITY, f64::min);
    if !z_min.is_finite() {
        return None;
    }
    let mut tris = Vec::with_capacity(b.tris.len());
    let (mut min_x, mut max_x, mut min_z, mut max_z) = (i32::MAX, i32::MIN, i32::MAX, i32::MIN);
    for t in &b.tris {
        let mut out = [[0.0f32; 3]; 3];
        for (k, v) in t.v.iter().enumerate() {
            let (lon, lat) = crs.to_lonlat(v[0], v[1]);
            let (x, z) = projector.world_xz(lon, lat)?;
            let y = ((v[2] - z_min) * unit) as f32 * scale;
            out[k] = [x, y, z];
            min_x = min_x.min(x.floor() as i32);
            max_x = max_x.max(x.ceil() as i32);
            min_z = min_z.min(z.floor() as i32);
            max_z = max_z.max(z.ceil() as i32);
        }
        tris.push((out, t.surface));
    }
    // Outside the generated world there is nothing to build on.
    if max_x < xzbbox.min_x()
        || min_x > xzbbox.max_x()
        || max_z < xzbbox.min_z()
        || min_z > xzbbox.max_z()
    {
        return None;
    }
    let h = hash_str(&b.id);
    let wall = [LIGHT_GRAY_CONCRETE, SMOOTH_STONE, STONE_BRICKS, BRICK][(h % 4) as usize];
    let roof = [DEEPSLATE_TILES, BROWN_TERRACOTTA, RED_TERRACOTTA][((h >> 8) % 3) as usize];
    Some(Placement {
        id: b.id.clone(),
        tris,
        min_x,
        max_x,
        min_z,
        max_z,
        wall,
        roof,
        flat_roof: [GRAY_CONCRETE, POLISHED_ANDESITE][((h >> 16) % 2) as usize],
    })
}

fn hash_str(s: &str) -> u32 {
    let mut h: u32 = 2_166_136_261;
    for b in s.bytes() {
        h ^= b as u32;
        h = h.wrapping_mul(16_777_619);
    }
    h
}

fn point_in_triangle_xz(px: f32, pz: f32, t: &[[f32; 3]; 3]) -> bool {
    let d = |a: &[f32; 3], b: &[f32; 3]| (b[0] - a[0]) * (pz - a[2]) - (b[2] - a[2]) * (px - a[0]);
    let (d1, d2, d3) = (d(&t[0], &t[1]), d(&t[1], &t[2]), d(&t[2], &t[0]));
    let neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(neg && pos)
}

fn centroid_xz(element: &ProcessedElement) -> Option<(i32, i32)> {
    let (mut sx, mut sz, mut n) = (0i64, 0i64, 0i64);
    let mut add = |x: i32, z: i32| {
        sx += x as i64;
        sz += z as i64;
        n += 1;
    };
    match element {
        ProcessedElement::Node(p) => add(p.x, p.z),
        ProcessedElement::Way(w) => w.nodes.iter().for_each(|p| add(p.x, p.z)),
        ProcessedElement::Relation(r) => r
            .members
            .iter()
            .flat_map(|m| m.way.nodes.iter())
            .for_each(|p| add(p.x, p.z)),
    }
    (n > 0).then(|| ((sx / n) as i32, (sz / n) as i32))
}

/// OSM buildings whose centre falls inside a city-model building are replaced by it; their
/// colour and material tags tint the model.
fn suppress_osm_buildings(
    elements: &[ProcessedElement],
    already_suppressed: &HashSet<(&'static str, u64)>,
    placements: &mut [Placement],
) -> HashSet<(&'static str, u64)> {
    const CELL: i32 = 32;
    let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (i, p) in placements.iter().enumerate() {
        for cx in p.min_x.div_euclid(CELL)..=p.max_x.div_euclid(CELL) {
            for cz in p.min_z.div_euclid(CELL)..=p.max_z.div_euclid(CELL) {
                grid.entry((cx, cz)).or_default().push(i);
            }
        }
    }

    let mut suppressed = HashSet::new();
    for element in elements {
        let key = (element.kind(), element.id());
        if already_suppressed.contains(&key) {
            continue;
        }
        let tags = element.tags();
        if !tags.contains_key("building") && !tags.contains_key("building:part") {
            continue;
        }
        let Some((x, z)) = centroid_xz(element) else {
            continue;
        };
        let Some(cands) = grid.get(&(x.div_euclid(CELL), z.div_euclid(CELL))) else {
            continue;
        };
        let hit = cands.iter().copied().find(|&i| {
            let p = &placements[i];
            p.bbox_contains(x, z) && p.covers(x as f32 + 0.5, z as f32 + 0.5)
        });
        let Some(i) = hit else { continue };
        suppressed.insert(key);

        let p = &mut placements[i];
        if let Some(c) = tags
            .get("building:colour")
            .and_then(|c| color_text_to_rgb_tuple(c))
        {
            p.wall = closest_block(c);
        } else if let Some(m) = tags.get("building:material") {
            match m.as_str() {
                "brick" => p.wall = BRICK,
                "stone" | "concrete" | "cement_block" => p.wall = LIGHT_GRAY_CONCRETE,
                _ => {}
            }
        }
        if let Some(c) = tags
            .get("roof:colour")
            .and_then(|c| color_text_to_rgb_tuple(c))
        {
            p.roof = closest_block(c);
            p.flat_roof = p.roof;
        }
    }
    suppressed
}

/// Floors are about three blocks apart at full scale; windows sit on every wall voxel of the
/// second block of a storey, leaving a pier every third column.
fn is_window(x: i32, z: i32, y_rel: i32, storey: i32) -> bool {
    storey >= 2 && y_rel >= storey && (y_rel % storey) == storey / 2 && (x + z).rem_euclid(3) != 0
}

fn voxelize_placement(p: &Placement, base_y: i32, scale: f64) -> Vec<([i32; 3], Block)> {
    let shader = |_: Option<&()>, _: [i32; 3], _: [f32; 3]| {};
    let mut classes: [DdaVoxelizer<()>; 3] = [
        DdaVoxelizer::new(),
        DdaVoxelizer::new(),
        DdaVoxelizer::new(),
    ];
    let mut flat_roof: HashSet<[i32; 2]> = HashSet::new();
    for (t, surface) in &p.tris {
        if !t.iter().all(|v| v.iter().all(|c| c.is_finite())) {
            continue;
        }
        let slot = match surface {
            Surface::Roof => 0,
            Surface::Wall | Surface::Other => 1,
            Surface::Ground => 2,
        };
        classes[slot].add_triangle(t, &shader);
        if slot == 0 && roof_normal_y(t) > 0.97 {
            for v in t {
                flat_roof.insert([v[0].round() as i32, v[2].round() as i32]);
            }
        }
    }
    let [roof_v, wall_v, ground_v] = classes;
    let roof_cells: HashSet<[i32; 3]> = roof_v.finalize().into_keys().collect();
    let wall_cells: HashSet<[i32; 3]> = wall_v.finalize().into_keys().collect();
    let ground_cells: HashSet<[i32; 3]> = ground_v.finalize().into_keys().collect();

    let storey = (3.0 * scale).round().max(1.0) as i32;
    let mut out: HashMap<[i32; 3], Block> = HashMap::new();
    for pos in ground_cells {
        out.insert(pos, p.wall);
    }
    for pos in wall_cells {
        let block = if is_window(pos[0], pos[2], pos[1], storey) {
            GLASS
        } else {
            p.wall
        };
        out.insert(pos, block);
    }
    // Roof tint: flat roofs get their own surface, pitched ones the tile colour.
    let flat: HashSet<[i32; 2]> = flat_roof;
    for pos in roof_cells {
        let block = if flat.contains(&[pos[0], pos[2]]) && !p.tris.is_empty() {
            p.flat_roof
        } else {
            p.roof
        };
        out.insert(pos, block);
    }
    out.into_iter()
        .map(|([x, y, z], b)| ([x, y + base_y, z], b))
        .collect()
}

fn roof_normal_y(t: &[[f32; 3]; 3]) -> f32 {
    let a = [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]];
    let b = [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]];
    let n = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len < 1e-9 {
        0.0
    } else {
        (n[1] / len).abs()
    }
}

pub fn place(editor: &mut WorldEditor, args: &Args, prescan: &PrescanResult, indices: &[usize]) {
    if indices.is_empty() {
        return;
    }
    println!(
        "{} Building {} city-model building{}...",
        "  [+]".bold(),
        indices.len(),
        if indices.len() == 1 { "" } else { "s" }
    );

    let grounds: Vec<i32> = indices
        .iter()
        .map(|&i| {
            let p = &prescan.placements[i];
            crate::models_3d::lowest_ground_in_bbox(editor, p.min_x, p.min_z, p.max_x, p.max_z)
        })
        .collect();
    let built: Vec<Vec<([i32; 3], Block)>> = indices
        .par_iter()
        .zip(grounds.par_iter())
        .map(|(&i, &ground)| voxelize_placement(&prescan.placements[i], ground, args.scale))
        .collect();

    let mut total = 0usize;
    for (voxels, &ground) in built.iter().zip(&grounds) {
        let mut all = voxels.clone();
        // Walls on a slope reach down to the terrain instead of hanging above it.
        let lowest: HashMap<(i32, i32), (i32, Block)> =
            voxels.iter().fold(HashMap::new(), |mut m, ([x, y, z], b)| {
                m.entry((*x, *z))
                    .and_modify(|e| {
                        if *y < e.0 {
                            *e = (*y, *b);
                        }
                    })
                    .or_insert((*y, *b));
                m
            });
        for ((x, z), (y, block)) in lowest {
            if y > ground {
                let terrain = editor.get_ground_level(x, z).min(y);
                for fy in terrain..y {
                    all.push(([x, fy, z], block));
                }
            }
        }
        total += all.len();
        crate::models_3d::stamp_voxels(editor, &all);
    }
    println!(
        "  Placed {} city-model buildings ({} blocks)",
        built.len().to_string().bright_white().bold(),
        total
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square_placement() -> Placement {
        // A 6 x 6 x 6 box with a flat roof, as one wall ring, a roof and a floor.
        let q = |a: [f32; 3], b: [f32; 3], c: [f32; 3], d: [f32; 3], s: Surface| {
            vec![([a, b, c], s), ([a, c, d], s)]
        };
        let mut tris = Vec::new();
        tris.extend(q(
            [0.0, 0.0, 0.0],
            [6.0, 0.0, 0.0],
            [6.0, 0.0, 6.0],
            [0.0, 0.0, 6.0],
            Surface::Ground,
        ));
        tris.extend(q(
            [0.0, 6.0, 0.0],
            [6.0, 6.0, 0.0],
            [6.0, 6.0, 6.0],
            [0.0, 6.0, 6.0],
            Surface::Roof,
        ));
        tris.extend(q(
            [0.0, 0.0, 0.0],
            [6.0, 0.0, 0.0],
            [6.0, 6.0, 0.0],
            [0.0, 6.0, 0.0],
            Surface::Wall,
        ));
        tris.extend(q(
            [0.0, 0.0, 6.0],
            [6.0, 0.0, 6.0],
            [6.0, 6.0, 6.0],
            [0.0, 6.0, 6.0],
            Surface::Wall,
        ));
        tris.extend(q(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 6.0],
            [0.0, 6.0, 6.0],
            [0.0, 6.0, 0.0],
            Surface::Wall,
        ));
        tris.extend(q(
            [6.0, 0.0, 0.0],
            [6.0, 0.0, 6.0],
            [6.0, 6.0, 6.0],
            [6.0, 6.0, 0.0],
            Surface::Wall,
        ));
        Placement {
            id: "box".to_string(),
            tris,
            min_x: 0,
            max_x: 6,
            min_z: 0,
            max_z: 6,
            wall: LIGHT_GRAY_CONCRETE,
            roof: RED_TERRACOTTA,
            flat_roof: GRAY_CONCRETE,
        }
    }

    #[test]
    fn box_voxelizes_to_walls_roof_and_floor() {
        let p = square_placement();
        let voxels = voxelize_placement(&p, 100, 1.0);
        assert!(!voxels.is_empty());
        let ys: Vec<i32> = voxels.iter().map(|(p, _)| p[1]).collect();
        assert_eq!(*ys.iter().min().unwrap(), 100);
        assert_eq!(*ys.iter().max().unwrap(), 106);
        assert!(voxels.iter().any(|(_, b)| *b == GRAY_CONCRETE), "flat roof");
        assert!(voxels.iter().any(|(_, b)| *b == GLASS), "windows");
    }

    #[test]
    fn covers_uses_the_footprint_not_the_bounding_box() {
        let p = square_placement();
        assert!(p.covers(3.0, 3.0));
        assert!(!p.covers(9.0, 3.0));
    }

    #[test]
    fn windows_leave_piers_and_skip_the_ground_floor() {
        assert!(!is_window(0, 0, 1, 3), "ground storey stays solid");
        assert!(is_window(1, 0, 4, 3));
        assert!(!is_window(3, 0, 4, 3), "every third column is a pier");
    }
}
