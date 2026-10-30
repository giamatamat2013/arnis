//! Automatic download of open city models for the generated area.
//!
//! Every provider covers one country or state. A provider whose coverage does not touch the
//! area is skipped without a request, so areas outside all of them cost nothing.

use super::crs::Crs;
use crate::coordinate_system::geographic::LLBBox;
use rayon::prelude::*;
use reqwest::blocking::Client;
use std::fs;
use std::path::PathBuf;
use std::sync::OnceLock;
use std::time::Duration;

const CACHE_SUBDIR: &str = "arnis/city_models";
const REQUEST_TIMEOUT_SECS: u64 = 60;
/// Pages of 100 buildings; bounds the download for a very large dense area.
const BAG_MAX_PAGES: usize = 300;
const NRW_MAX_TILES: usize = 64;

pub fn cache_root() -> PathBuf {
    dirs::cache_dir()
        .map(|d| d.join(CACHE_SUBDIR))
        .unwrap_or_else(|| PathBuf::from("./.arnis_city_models_cache"))
}

fn client() -> Result<&'static Client, String> {
    static CLIENT: OnceLock<Result<Client, String>> = OnceLock::new();
    CLIENT
        .get_or_init(|| {
            Client::builder()
                .timeout(Duration::from_secs(REQUEST_TIMEOUT_SECS))
                .user_agent(concat!(
                    "Arnis/",
                    env!("CARGO_PKG_VERSION"),
                    " (+https://github.com/louis-e/arnis)"
                ))
                .build()
                .map_err(|e| e.to_string())
        })
        .as_ref()
        .map_err(Clone::clone)
}

fn get_bytes(url: &str, accept: Option<&str>) -> Result<Option<Vec<u8>>, String> {
    let _permit = crate::net::request_permit();
    let mut req = client()?.get(url);
    if let Some(a) = accept {
        req = req.header("Accept", a);
    }
    let resp = req.send().map_err(|e| crate::net::failure_kind(&e))?;
    if resp.status() == reqwest::StatusCode::NOT_FOUND {
        return Ok(None);
    }
    if !resp.status().is_success() {
        return Err(format!("HTTP {}", resp.status()));
    }
    resp.bytes()
        .map(|b| Some(b.to_vec()))
        .map_err(|e| crate::net::failure_kind(&e))
}

/// Files (CityJSONSeq or CityGML) holding the city models that cover the area. Download
/// failures are reported and skipped, so generation always continues without them.
pub fn models_for(llbbox: &LLBBox) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for (name, result) in [
        ("3D BAG (Netherlands)", netherlands(llbbox)),
        ("NRW LoD2 (Germany)", nrw(llbbox)),
    ] {
        match result {
            Ok(mut files) => out.append(&mut files),
            Err(e) => eprintln!("Warning: city model download from {name} failed: {e}"),
        }
    }
    out
}

/// Corner-based range of the area in a projected system, in that system's unit.
fn projected_range(crs: Crs, bbox: &LLBBox) -> (f64, f64, f64, f64) {
    let (min, max) = (bbox.min(), bbox.max());
    let corners = [
        (min.lng(), min.lat()),
        (min.lng(), max.lat()),
        (max.lng(), min.lat()),
        (max.lng(), max.lat()),
    ];
    corners.iter().map(|&(lo, la)| crs.from_lonlat(lo, la)).fold(
        (f64::MAX, f64::MAX, f64::MIN, f64::MIN),
        |(x0, y0, x1, y1), (x, y)| (x0.min(x), y0.min(y), x1.max(x), y1.max(y)),
    )
}

fn intersects(bbox: &LLBBox, lon: (f64, f64), lat: (f64, f64)) -> bool {
    bbox.min().lng() <= lon.1
        && bbox.max().lng() >= lon.0
        && bbox.min().lat() <= lat.1
        && bbox.max().lat() >= lat.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models_3d::city_models::parse::{load_path, LoadOptions};

    fn bbox(min_lat: f64, min_lng: f64, max_lat: f64, max_lng: f64) -> LLBBox {
        LLBBox::new(min_lat, min_lng, max_lat, max_lng).unwrap()
    }

    #[test]
    fn areas_outside_every_provider_download_nothing() {
        assert!(models_for(&bbox(40.74, -74.0, 40.75, -73.99)).is_empty());
    }

    /// Needs network access: `cargo test live_ -- --ignored`.
    #[test]
    #[ignore]
    fn live_amsterdam_loads() {
        let files = netherlands(&bbox(52.3700, 4.8890, 52.3720, 4.8920)).unwrap();
        let loaded = load_path(&files[0], LoadOptions::default()).unwrap();
        let n: usize = loaded.iter().map(|l| l.buildings.len()).sum();
        assert!(n > 5, "only {n} buildings");
    }

    #[test]
    #[ignore]
    fn live_cologne_loads() {
        let files = nrw(&bbox(50.9400, 6.9570, 50.9420, 6.9600)).unwrap();
        let loaded = load_path(&files[0], LoadOptions::default()).unwrap();
        let n: usize = loaded.iter().map(|l| l.buildings.len()).sum();
        assert!(n > 5, "only {n} buildings");
    }
}

// ------------------------------------------------------------------ Netherlands

/// 3D BAG serves its buildings as CityJSON features over an OGC API, 100 per page, with the
/// bounding box in Dutch RD metres.
fn netherlands(bbox: &LLBBox) -> Result<Vec<PathBuf>, String> {
    if !intersects(bbox, (3.2, 7.3), (50.7, 53.7)) {
        return Ok(Vec::new());
    }
    let (x0, y0, x1, y1) = projected_range(Crs::AmersfoortRd, bbox);
    let dir = cache_root().join("nl");
    let path = dir.join(format!("{:.0}_{:.0}_{:.0}_{:.0}.jsonl", x0, y0, x1, y1));
    if path.is_file() {
        return Ok(vec![path]);
    }

    // The server numbers features from 1 and rejects an explicit `offset=0`.
    let url = |page: usize| {
        let base = format!(
            "https://api.3dbag.nl/collections/pand/items?bbox={x0:.1},{y0:.1},{x1:.1},{y1:.1}&limit=100"
        );
        if page == 0 {
            base
        } else {
            format!("{base}&offset={}", page * 100 + 1)
        }
    };
    let page = |index: usize| -> Result<serde_json::Value, String> {
        let bytes = get_bytes(&url(index), Some("application/city+json"))?
            .ok_or("collection not found")?;
        serde_json::from_slice(&bytes).map_err(|e| e.to_string())
    };

    let first = page(0)?;
    let matched = first
        .get("numberMatched")
        .and_then(serde_json::Value::as_u64)
        .unwrap_or(0) as usize;
    if matched == 0 {
        return Ok(Vec::new());
    }
    println!("  Downloading 3D BAG: {matched} buildings...");
    let pages = matched.div_ceil(100).min(BAG_MAX_PAGES);
    if matched.div_ceil(100) > pages {
        eprintln!(
            "Warning: area has {matched} buildings, only the first {} get a real 3D model",
            pages * 100
        );
    }

    let header = first
        .get("metadata")
        .cloned()
        .ok_or("3D BAG response has no metadata")?;
    let rest: Vec<Result<serde_json::Value, String>> = (1..pages)
        .into_par_iter()
        .map(page)
        .collect();

    let mut text = header.to_string();
    for doc in std::iter::once(Ok(first)).chain(rest) {
        let doc = doc?;
        for feature in doc
            .get("features")
            .and_then(serde_json::Value::as_array)
            .into_iter()
            .flatten()
        {
            text.push('\n');
            text.push_str(&feature.to_string());
        }
    }
    fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    fs::write(&path, text).map_err(|e| e.to_string())?;
    Ok(vec![path])
}

// ------------------------------------------------------------------ North Rhine-Westphalia

/// One CityGML file per 1 km UTM 32 tile, named after its south-west corner in kilometres.
fn nrw(bbox: &LLBBox) -> Result<Vec<PathBuf>, String> {
    if !intersects(bbox, (5.8, 9.5), (50.3, 52.6)) {
        return Ok(Vec::new());
    }
    let utm32 = Crs::Utm {
        zone: 32,
        north: true,
    };
    let (x0, y0, x1, y1) = projected_range(utm32, bbox);
    let (e0, e1) = ((x0 / 1000.0).floor() as i64, (x1 / 1000.0).floor() as i64);
    let (n0, n1) = ((y0 / 1000.0).floor() as i64, (y1 / 1000.0).floor() as i64);
    let tiles: Vec<(i64, i64)> = (e0..=e1)
        .flat_map(|e| (n0..=n1).map(move |n| (e, n)))
        .collect();
    if tiles.len() > NRW_MAX_TILES {
        eprintln!(
            "Warning: area spans {} NRW tiles, only the first {NRW_MAX_TILES} get a real 3D model",
            tiles.len()
        );
    }

    let dir = cache_root().join("nrw");
    fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let results: Vec<Result<Option<PathBuf>, String>> = tiles
        .into_par_iter()
        .take(NRW_MAX_TILES)
        .map(|(e, n)| {
            let name = format!("LoD2_32_{e}_{n}_1_NW.gml");
            let path = dir.join(&name);
            if path.is_file() {
                return Ok(Some(path));
            }
            let url = format!(
                "https://www.opengeodata.nrw.de/produkte/geobasis/3dg/lod2_gml/lod2_gml/{name}"
            );
            match get_bytes(&url, None)? {
                Some(bytes) => {
                    fs::write(&path, bytes).map_err(|err| err.to_string())?;
                    Ok(Some(path))
                }
                None => Ok(None),
            }
        })
        .collect();

    let mut files = Vec::new();
    let mut first_error = None;
    for r in results {
        match r {
            Ok(Some(p)) => files.push(p),
            Ok(None) => {}
            Err(e) => first_error = first_error.or(Some(e)),
        }
    }
    match (files.is_empty(), first_error) {
        (true, Some(e)) => Err(e),
        _ => Ok(files),
    }
}
