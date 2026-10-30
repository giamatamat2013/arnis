//! Coordinate systems used by open city models, converted to WGS84 longitude/latitude
//! without a projection library.

use std::f64::consts::PI;

const WGS84_A: f64 = 6_378_137.0;
const WGS84_F: f64 = 1.0 / 298.257_223_563;
const US_FOOT_M: f64 = 1200.0 / 3937.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Crs {
    /// Degrees; `lat_first` for axis orders that put latitude first.
    Geographic {
        lat_first: bool,
    },
    WebMercator,
    /// UTM on a GRS80/WGS84 ellipsoid (ETRS89 and WGS84 UTM zones).
    Utm {
        zone: u8,
        north: bool,
    },
    /// Dutch Rijksdriehoekstelsel (EPSG:28992, also the horizontal part of 7415).
    AmersfoortRd,
    /// New York State Plane, Long Island, US survey feet (EPSG:2263).
    NyLongIslandFt,
}

impl Crs {
    /// Maps an EPSG code to a supported system.
    pub fn from_epsg(code: u32) -> Option<Crs> {
        match code {
            4326 | 4979 | 4258 | 4937 | 6318 | 4269 => Some(Crs::Geographic { lat_first: false }),
            3857 | 900_913 => Some(Crs::WebMercator),
            28992 | 7415 => Some(Crs::AmersfoortRd),
            2263 | 6539 => Some(Crs::NyLongIslandFt),
            32_601..=32_660 => Some(Crs::Utm {
                zone: (code - 32_600) as u8,
                north: true,
            }),
            32_701..=32_760 => Some(Crs::Utm {
                zone: (code - 32_700) as u8,
                north: false,
            }),
            // ETRS89 / UTM 28N-38N.
            25_828..=25_838 => Some(Crs::Utm {
                zone: (code - 25_800) as u8,
                north: true,
            }),
            // ETRS89 / UTM 32N and 33N with the N-E axis order of the German "3044" family.
            3044 => Some(Crs::Utm {
                zone: 32,
                north: true,
            }),
            3045 => Some(Crs::Utm {
                zone: 33,
                north: true,
            }),
            // NAD83 UTM 1N-23N and the German DHDN/ETRS89 realisations on 32/33.
            26_901..=26_923 => Some(Crs::Utm {
                zone: (code - 26_900) as u8,
                north: true,
            }),
            _ => None,
        }
    }

    /// Multiplier from the horizontal unit to metres.
    pub fn unit_m(&self) -> f64 {
        match self {
            Crs::NyLongIslandFt => US_FOOT_M,
            _ => 1.0,
        }
    }

    /// Converts a horizontal coordinate pair to (longitude, latitude) in degrees.
    pub fn to_lonlat(&self, x: f64, y: f64) -> (f64, f64) {
        match *self {
            Crs::Geographic { lat_first } => {
                if lat_first {
                    (y, x)
                } else {
                    (x, y)
                }
            }
            Crs::WebMercator => {
                let lon = x / WGS84_A * 180.0 / PI;
                let lat = (2.0 * (y / WGS84_A).exp().atan() - PI / 2.0) * 180.0 / PI;
                (lon, lat)
            }
            Crs::Utm { zone, north } => utm_to_lonlat(x, y, zone, north),
            Crs::AmersfoortRd => rd_to_lonlat(x, y),
            Crs::NyLongIslandFt => ny_li_to_lonlat(x, y),
        }
    }
}

/// Reads the EPSG number out of `EPSG:7415`, `urn:ogc:def:crs:EPSG::25832`,
/// `https://www.opengis.net/def/crs/EPSG/0/7415` and compound forms like `EPSG:7415+...`.
pub fn parse_epsg(text: &str) -> Option<u32> {
    let upper = text.to_ascii_uppercase();
    let start = upper.find("EPSG")?;
    let rest = &upper[start + 4..];
    // A compound CRS lists the next system after another "EPSG"; only the first counts.
    let rest = rest.split("EPSG").next().unwrap_or(rest);
    // The URL form is EPSG/0/7415, where 0 is the version.
    rest.split(|c: char| !c.is_ascii_digit())
        .filter_map(|t| t.parse::<u32>().ok())
        .find(|&code| code != 0)
}

/// Reads `--city-model-crs`: `EPSG:25832`, a bare `25832`, or a full CRS URI.
pub fn crs_from_arg(text: &str) -> Result<Crs, String> {
    let trimmed = text.trim();
    let code = trimmed
        .parse::<u32>()
        .ok()
        .or_else(|| parse_epsg(trimmed))
        .ok_or_else(|| format!("--city-model-crs: cannot read an EPSG code from \"{text}\""))?;
    Crs::from_epsg(code).ok_or_else(|| {
        format!(
            "--city-model-crs: EPSG:{code} is not supported (WGS84, Web Mercator, UTM/ETRS89 \
             zones, Dutch RD 28992/7415, NY Long Island 2263)"
        )
    })
}

/// True for GML `srsName`s that list latitude first (`urn:ogc:def:crs:EPSG::4326`).
pub fn urn_is_lat_first(text: &str, crs: &Crs) -> bool {
    matches!(crs, Crs::Geographic { .. }) && text.to_ascii_lowercase().starts_with("urn:")
}

fn utm_to_lonlat(easting: f64, northing: f64, zone: u8, north: bool) -> (f64, f64) {
    let n = WGS84_F / (2.0 - WGS84_F);
    let a_big = WGS84_A / (1.0 + n) * (1.0 + n * n / 4.0 + n.powi(4) / 64.0);
    let k0 = 0.9996;
    let e0 = 500_000.0;
    let n0 = if north { 0.0 } else { 10_000_000.0 };

    let xi_p = (northing - n0) / (k0 * a_big);
    let eta_p = (easting - e0) / (k0 * a_big);

    let beta = [
        n / 2.0 - 2.0 * n * n / 3.0 + 37.0 * n.powi(3) / 96.0,
        n * n / 48.0 + n.powi(3) / 15.0,
        17.0 * n.powi(3) / 480.0,
    ];
    let delta = [
        2.0 * n - 2.0 * n * n / 3.0 - 2.0 * n.powi(3),
        7.0 * n * n / 3.0 - 8.0 * n.powi(3) / 5.0,
        56.0 * n.powi(3) / 15.0,
    ];

    let mut xi = xi_p;
    let mut eta = eta_p;
    for (j, b) in beta.iter().enumerate() {
        let k = 2.0 * (j as f64 + 1.0);
        xi -= b * (k * xi_p).sin() * (k * eta_p).cosh();
        eta -= b * (k * xi_p).cos() * (k * eta_p).sinh();
    }
    let chi = (xi.sin() / eta.cosh()).asin();
    let mut lat = chi;
    for (j, d) in delta.iter().enumerate() {
        lat += d * (2.0 * (j as f64 + 1.0) * chi).sin();
    }
    let lon0 = ((zone as f64 - 1.0) * 6.0 - 180.0 + 3.0).to_radians();
    let lon = lon0 + eta.sinh().atan2(xi.cos());
    (lon.to_degrees(), lat.to_degrees())
}

/// Dutch RD to WGS84 by the Schreutelkamp/Strang van Hees series (about a metre).
fn rd_to_lonlat(x: f64, y: f64) -> (f64, f64) {
    let dx = (x - 155_000.0) * 1e-5;
    let dy = (y - 463_000.0) * 1e-5;
    let som_n = 3235.65389 * dy
        - 32.58297 * dx.powi(2)
        - 0.24750 * dy.powi(2)
        - 0.84978 * dx.powi(2) * dy
        - 0.06550 * dy.powi(3)
        - 0.01709 * dx.powi(2) * dy.powi(2)
        - 0.00738 * dx
        + 0.00530 * dx.powi(4)
        - 0.00039 * dx.powi(2) * dy.powi(3)
        + 0.00033 * dx.powi(4) * dy
        - 0.00012 * dx * dy;
    let som_e = 5260.52916 * dx + 105.94684 * dx * dy + 2.45656 * dx * dy.powi(2)
        - 0.81885 * dx.powi(3)
        + 0.05594 * dx * dy.powi(3)
        - 0.05607 * dx.powi(3) * dy
        + 0.01199 * dy
        - 0.00256 * dx.powi(3) * dy.powi(2)
        + 0.00128 * dx * dy.powi(4)
        + 0.00022 * dy.powi(2)
        - 0.00022 * dx.powi(2)
        + 0.00026 * dx.powi(5);
    let lat = 52.155_174_40 + som_n / 3600.0;
    let lon = 5.387_206_21 + som_e / 3600.0;
    (lon, lat)
}

/// Lambert Conformal Conic (2SP), NAD83 / New York Long Island, input in US feet.
fn ny_li_to_lonlat(x_ft: f64, y_ft: f64) -> (f64, f64) {
    let e2: f64 = 0.006_694_380_022_90;
    let e = e2.sqrt();
    let a = WGS84_A;
    let (lat1, lat2, lat0, lon0) = (
        41.033_333_333_f64.to_radians(),
        40.666_666_667_f64.to_radians(),
        40.166_666_667_f64.to_radians(),
        (-74.0_f64).to_radians(),
    );
    let m = |p: f64| p.cos() / (1.0 - e2 * p.sin().powi(2)).sqrt();
    let t = |p: f64| {
        (PI / 4.0 - p / 2.0).tan() / ((1.0 - e * p.sin()) / (1.0 + e * p.sin())).powf(e / 2.0)
    };
    let n = (m(lat1).ln() - m(lat2).ln()) / (t(lat1).ln() - t(lat2).ln());
    let f = m(lat1) / (n * t(lat1).powf(n));
    let rho0 = a * f * t(lat0).powf(n);

    let xm = x_ft * US_FOOT_M - 300_000.0;
    let ym = y_ft * US_FOOT_M;
    let rho = n.signum() * (xm * xm + (rho0 - ym).powi(2)).sqrt();
    let theta = (n.signum() * xm).atan2(n.signum() * (rho0 - ym));
    let tt = (rho / (a * f)).powf(1.0 / n);
    let mut lat = PI / 2.0 - 2.0 * tt.atan();
    for _ in 0..8 {
        lat = PI / 2.0
            - 2.0 * (tt * ((1.0 - e * lat.sin()) / (1.0 + e * lat.sin())).powf(e / 2.0)).atan();
    }
    let lon = theta / n + lon0;
    (lon.to_degrees(), lat.to_degrees())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_epsg_spellings() {
        assert_eq!(parse_epsg("EPSG:7415"), Some(7415));
        assert_eq!(parse_epsg("urn:ogc:def:crs:EPSG::25832"), Some(25832));
        assert_eq!(
            parse_epsg("https://www.opengis.net/def/crs/EPSG/0/7415"),
            Some(7415)
        );
        assert_eq!(parse_epsg("urn:adv:crs:ETRS89_UTM32"), None);
    }

    #[test]
    fn rd_origin_is_amersfoort() {
        let (lon, lat) = Crs::AmersfoortRd.to_lonlat(155_000.0, 463_000.0);
        assert!((lat - 52.155_17).abs() < 1e-4 && (lon - 5.387_2).abs() < 1e-4);
    }

    #[test]
    fn utm_central_meridian_on_the_equator() {
        let (lon, lat) = Crs::Utm {
            zone: 32,
            north: true,
        }
        .to_lonlat(500_000.0, 0.0);
        assert!((lon - 9.0).abs() < 1e-9 && lat.abs() < 1e-9);
    }

    #[test]
    fn utm32_cologne_cathedral() {
        // Cologne Cathedral, about 50.9413 N 6.9583 E.
        let (lon, lat) = Crs::Utm {
            zone: 32,
            north: true,
        }
        .to_lonlat(356_850.0, 5_645_850.0);
        assert!(
            (lat - 50.941).abs() < 0.01 && (lon - 6.958).abs() < 0.01,
            "{lat} {lon}"
        );
    }

    #[test]
    fn new_york_state_plane_near_the_empire_state_building() {
        // Empire State Building, about 40.7484 N 73.9857 W, at roughly 988,200 / 211,900 ft.
        let (lon, lat) = Crs::NyLongIslandFt.to_lonlat(988_200.0, 211_900.0);
        assert!(
            (lat - 40.748).abs() < 0.01 && (lon + 73.986).abs() < 0.01,
            "{lat} {lon}"
        );
    }

    #[test]
    fn web_mercator_origin() {
        let (lon, lat) = Crs::WebMercator.to_lonlat(0.0, 0.0);
        assert!(lon.abs() < 1e-9 && lat.abs() < 1e-9);
    }
}
