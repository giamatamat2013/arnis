//! Ear-clipping triangulation of a planar 3D polygon ring.

type P3 = [f64; 3];

/// Triangles (as index triples into `ring`) covering the polygon. A closing vertex equal to
/// the first is ignored. Degenerate rings return nothing.
pub fn triangulate_ring(ring: &[P3]) -> Vec<[usize; 3]> {
    let mut n = ring.len();
    if n > 1 && ring[0] == ring[n - 1] {
        n -= 1;
    }
    if n < 3 {
        return Vec::new();
    }
    if n == 3 {
        return vec![[0, 1, 2]];
    }

    // Newell normal, then drop the dominant axis to work in 2D.
    let mut normal = [0.0f64; 3];
    for i in 0..n {
        let (a, b) = (ring[i], ring[(i + 1) % n]);
        normal[0] += (a[1] - b[1]) * (a[2] + b[2]);
        normal[1] += (a[2] - b[2]) * (a[0] + b[0]);
        normal[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    let abs = normal.map(f64::abs);
    let drop = if abs[0] >= abs[1] && abs[0] >= abs[2] {
        0
    } else if abs[1] >= abs[2] {
        1
    } else {
        2
    };
    if abs[drop] < 1e-12 {
        return Vec::new();
    }
    let (ua, va) = match drop {
        0 => (1, 2),
        1 => (2, 0),
        _ => (0, 1),
    };
    // Local origin keeps large projected coordinates (UTM) precise.
    let o = ring[0];
    let pts: Vec<[f64; 2]> = (0..n)
        .map(|i| [ring[i][ua] - o[ua], ring[i][va] - o[va]])
        .collect();

    let area: f64 = (0..n)
        .map(|i| {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    let mut idx: Vec<usize> = (0..n).collect();
    if area < 0.0 {
        idx.reverse();
    }

    let cross = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let inside = |p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        cross(a, b, p) >= 0.0 && cross(b, c, p) >= 0.0 && cross(c, a, p) >= 0.0
    };

    let mut tris = Vec::with_capacity(n - 2);
    let mut guard = 0usize;
    while idx.len() > 3 && guard < n * n + 8 {
        guard += 1;
        let m = idx.len();
        let mut clipped = false;
        for i in 0..m {
            let (ip, ic, inx) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
            let (a, b, c) = (pts[ip], pts[ic], pts[inx]);
            if cross(a, b, c) <= 1e-12 {
                continue;
            }
            let blocked = idx.iter().any(|&j| {
                j != ip && j != ic && j != inx && {
                    let p = pts[j];
                    p != a && p != b && p != c && inside(p, a, b, c)
                }
            });
            if blocked {
                continue;
            }
            tris.push([ip, ic, inx]);
            idx.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            // Self-touching or collinear leftovers: drop a collinear vertex, else fan the rest.
            let m = idx.len();
            if let Some(i) = (0..m).find(|&i| {
                cross(
                    pts[idx[(i + m - 1) % m]],
                    pts[idx[i]],
                    pts[idx[(i + 1) % m]],
                )
                .abs()
                    <= 1e-12
            }) {
                idx.remove(i);
            } else {
                break;
            }
        }
    }
    if idx.len() == 3 {
        tris.push([idx[0], idx[1], idx[2]]);
    } else if idx.len() > 3 {
        for k in 1..idx.len() - 1 {
            tris.push([idx[0], idx[k], idx[k + 1]]);
        }
    }
    tris
}

#[cfg(test)]
mod tests {
    use super::*;

    fn area_xy(ring: &[P3], tris: &[[usize; 3]]) -> f64 {
        tris.iter()
            .map(|t| {
                let (a, b, c) = (ring[t[0]], ring[t[1]], ring[t[2]]);
                ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs() / 2.0
            })
            .sum()
    }

    #[test]
    fn square_makes_two_triangles() {
        let ring = [
            [0.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [4.0, 4.0, 0.0],
            [0.0, 4.0, 0.0],
        ];
        let tris = triangulate_ring(&ring);
        assert_eq!(tris.len(), 2);
        assert!((area_xy(&ring, &tris) - 16.0).abs() < 1e-9);
    }

    #[test]
    fn l_shape_keeps_its_area() {
        let ring = [
            [0.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [4.0, 2.0, 0.0],
            [2.0, 2.0, 0.0],
            [2.0, 4.0, 0.0],
            [0.0, 4.0, 0.0],
        ];
        let tris = triangulate_ring(&ring);
        assert_eq!(tris.len(), 4);
        assert!((area_xy(&ring, &tris) - 12.0).abs() < 1e-9);
    }

    #[test]
    fn closing_vertex_and_clockwise_order_are_handled() {
        let ring = [
            [0.0, 0.0, 5.0],
            [0.0, 4.0, 5.0],
            [4.0, 4.0, 5.0],
            [4.0, 0.0, 5.0],
            [0.0, 0.0, 5.0],
        ];
        let tris = triangulate_ring(&ring);
        assert_eq!(tris.len(), 2);
        assert!((area_xy(&ring, &tris) - 16.0).abs() < 1e-9);
    }

    #[test]
    fn vertical_wall_triangulates_in_its_own_plane() {
        let ring = [
            [0.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [5.0, 0.0, 3.0],
            [0.0, 0.0, 3.0],
        ];
        assert_eq!(triangulate_ring(&ring).len(), 2);
    }
}
