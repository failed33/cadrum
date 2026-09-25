//! Integration tests for `Edge` query APIs — `EdgeStruct::project` and the
//! `wire_project` iterator idiom for an ordered edge list.
//!
//! Covers single `Edge` and multi-edge wire (`Vec<Edge>`) paths across a
//! mix of curve kinds (line, circle, polygon, B-spline). The expected
//! closest point is computed analytically from the curve definition and
//! compared to the FFI result.

use cadrum::{BSplineEnd, Edge};
use glam::DVec3;

const TOL: f64 = 1e-6;

fn approx_eq(a: DVec3, b: DVec3, tol: f64) -> bool {
	(a - b).length() < tol
}

/// Project a point onto a wire (= ordered edge list) by taking the nearest
/// per-edge projection. This is the iterator idiom that replaces the removed
/// `Wire::project` collection method.
fn wire_project(wire: &[Edge], p: DVec3) -> (DVec3, DVec3) {
	wire.iter().map(|e| e.project(p)).min_by(|(a, _), (b, _)| (*a - p).length_squared().partial_cmp(&(*b - p).length_squared()).unwrap()).unwrap_or((DVec3::ZERO, DVec3::ZERO))
}

#[test]
fn project_on_line_midpoint() {
	// Line from -X to +X along X axis; query above the midpoint -> projects to origin.
	let e = Edge::line(DVec3::new(-1.0, 0.0, 0.0), DVec3::new(1.0, 0.0, 0.0)).unwrap();
	let (cp, tg) = e.project(DVec3::new(0.0, 1.0, 0.0));
	assert!(approx_eq(cp, DVec3::ZERO, TOL), "cp={cp:?}");
	// Tangent is ±X (normalized).
	assert!((tg.x.abs() - 1.0).abs() < TOL && tg.y.abs() < TOL && tg.z.abs() < TOL, "tg={tg:?}");
}

#[test]
fn project_on_circle_returns_radius() {
	// Unit circle in XY plane; project (2, 0, 0) -> expect (1, 0, 0).
	let e = Edge::circle(1.0, DVec3::Z).unwrap();
	let (cp, tg) = e.project(DVec3::new(2.0, 0.0, 0.0));
	assert!(approx_eq(cp, DVec3::new(1.0, 0.0, 0.0), TOL), "cp={cp:?}");
	// At (1,0,0) the unit-tangent to a CCW-parameterized circle is ±Y.
	assert!(tg.x.abs() < TOL && (tg.y.abs() - 1.0).abs() < TOL && tg.z.abs() < TOL, "tg={tg:?}");
	// Off-plane query still lands on the circle in XY.
	let (cp2, _) = e.project(DVec3::new(3.0, 0.0, 5.0));
	assert!(approx_eq(cp2, DVec3::new(1.0, 0.0, 0.0), TOL), "cp2={cp2:?}");
}

#[test]
fn project_on_polygon_picks_nearest_edge() {
	// Closed square in XY plane: edges (±1, ±1, 0). Query point near +X edge.
	let square = Edge::polygon([DVec3::new(1.0, 1.0, 0.0), DVec3::new(-1.0, 1.0, 0.0), DVec3::new(-1.0, -1.0, 0.0), DVec3::new(1.0, -1.0, 0.0)].iter()).unwrap();
	// (2, 0, 0) is closest to the right edge x=1, y∈[-1,1] -> (1, 0, 0).
	let (cp, _) = wire_project(&square, DVec3::new(2.0, 0.0, 0.0));
	assert!(approx_eq(cp, DVec3::new(1.0, 0.0, 0.0), TOL), "cp={cp:?}");
	// (-2, -3, 0) is closest to the corner (-1, -1, 0).
	let (cp2, _) = wire_project(&square, DVec3::new(-2.0, -3.0, 0.0));
	assert!(approx_eq(cp2, DVec3::new(-1.0, -1.0, 0.0), TOL), "cp2={cp2:?}");
}

#[test]
fn project_on_bspline_converges_to_interpolant() {
	// Periodic cubic B-spline through four XY ring points. Origin should
	// project somewhere on the ring; its distance is roughly the ring's
	// mean radius (≈1.0 here since all control points are unit-distant).
	let pts = [DVec3::new(1.0, 0.0, 0.0), DVec3::new(0.0, 1.0, 0.0), DVec3::new(-1.0, 0.0, 0.0), DVec3::new(0.0, -1.0, 0.0)];
	let e = Edge::bspline(pts.iter(), BSplineEnd::Periodic).unwrap();
	let (cp, tg) = e.project(DVec3::ZERO);
	// A periodic cubic B-spline interpolating 4 unit-distance points is
	// not a perfect circle — it "cuts the corner" between knots, so the
	// closest-to-origin point sits strictly inside the unit circle.
	// Confirm the projection is on the curve (not at origin) and within
	// the sensible envelope (chord midpoint = √0.5 ≈ 0.707 .. unit = 1.0).
	let r = cp.length();
	assert!((0.7..=1.0).contains(&r), "radius out of envelope: {r}");
	// Tangent is unit-length.
	assert!((tg.length() - 1.0).abs() < TOL, "|tg|={}", tg.length());
}

#[test]
fn bezier_passes_its_end_poles_and_its_exact_apex() {
	// Symmetric cubic: B(0.5) = (P0 + 3 P1 + 3 P2 + P3) / 8 = (1, 0.75, 0), the apex.
	let poles = [DVec3::ZERO, DVec3::new(0.0, 1.0, 0.0), DVec3::new(2.0, 1.0, 0.0), DVec3::new(2.0, 0.0, 0.0)];
	let e = Edge::bezier(poles.iter()).unwrap();
	assert!(approx_eq(e.start_point(), poles[0], TOL) && approx_eq(e.end_point(), poles[3], TOL));
	let (cp, _) = e.project(DVec3::new(1.0, 2.0, 0.0));
	assert!(approx_eq(cp, DVec3::new(1.0, 0.75, 0.0), TOL), "cp={cp:?}");
	// A handle collapsed onto its end pole is a valid control point.
	assert!(Edge::bezier([DVec3::ZERO, DVec3::ZERO, DVec3::X, DVec3::X * 2.0].iter()).is_ok());
	assert!(Edge::bezier([DVec3::ZERO].iter()).is_err());
}

#[test]
fn at_length_states_the_fraction_of_the_parameter_range_it_reached() {
	let line = Edge::line(DVec3::ZERO, DVec3::new(8.0, 0.0, 0.0)).unwrap();
	let (point, _, parameter) = line.at_length(2.0).unwrap();
	assert!(approx_eq(point, DVec3::new(2.0, 0.0, 0.0), TOL), "{point:?}");
	assert!((parameter - 0.25).abs() < TOL, "{parameter}");
	let poles = [DVec3::ZERO, DVec3::new(0.0, 1.0, 0.0), DVec3::new(2.0, 1.0, 0.0), DVec3::new(2.0, 0.0, 0.0)];
	let (start, _, first) = Edge::bezier(poles.iter()).unwrap().at_length(0.0).unwrap();
	assert!(approx_eq(start, poles[0], TOL) && first.abs() < TOL, "{first}");
}
