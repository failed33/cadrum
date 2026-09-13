//! Integration tests for `Solid::extrude`, including profiles with holes.
//!
//! A profile is a flat edge list that `Edge::loops` splits into closed loops:
//! the first bounds the solid, the rest become holes. Volumes are checked
//! against the analytic value so a hole that failed to cut shows up as a
//! mismatch rather than a shape that merely looks right.

use cadrum::{Edge, Solid};
use glam::DVec3;
use std::f64::consts::PI;

const TOL: f64 = 1e-6;

/// Square plate of side `side` in the XY plane, centred on the origin.
fn plate(side: f64) -> Vec<Edge> {
	let h = side / 2.0;
	Edge::polygon(&[DVec3::new(-h, -h, 0.0), DVec3::new(h, -h, 0.0), DVec3::new(h, h, 0.0), DVec3::new(-h, h, 0.0)]).unwrap()
}

fn bore(radius: f64, x: f64) -> Vec<Edge> {
	vec![Edge::circle(radius, DVec3::Z).unwrap().translate(DVec3::X * x)]
}

#[test]
fn extrude_plain_plate() {
	let solid = Solid::extrude(&plate(10.0), DVec3::Z * 3.0).expect("plate");
	assert!((solid.volume() - 300.0).abs() < TOL, "volume={}", solid.volume());
}

#[test]
fn extrude_bore_removes_volume() {
	let solid = Solid::extrude(&[plate(10.0), bore(1.0, 0.0)].concat(), DVec3::Z * 3.0).expect("bored plate");
	let expected = (100.0 - PI) * 3.0;
	assert!((solid.volume() - expected).abs() < TOL, "volume={} expected={expected}", solid.volume());
}

#[test]
fn extrude_bore_adds_one_face_and_opens_the_axis() {
	let height = 2.0;
	let plain = Solid::extrude(&plate(8.0), DVec3::Z * height).unwrap();
	let bored = Solid::extrude(&[plate(8.0), bore(1.5, 0.0)].concat(), DVec3::Z * height).unwrap();
	assert_eq!(bored.iter_face().count(), plain.iter_face().count() + 1, "the bore wall is one extra face");
	assert!(plain.contains(DVec3::new(0.0, 0.0, height / 2.0)));
	assert!(!bored.contains(DVec3::new(0.0, 0.0, height / 2.0)), "the bore axis must be outside the material");
}

#[test]
fn extrude_two_bores() {
	let profile = [plate(10.0), bore(0.5, -3.0), bore(0.5, 3.0)].concat();
	let solid = Solid::extrude(&profile, DVec3::Z).expect("two bores");
	let expected = 100.0 - 2.0 * PI * 0.25;
	assert!((solid.volume() - expected).abs() < TOL, "volume={}", solid.volume());
}

#[test]
fn extrude_rejects_a_profile_that_never_closes() {
	let open = [Edge::line(DVec3::ZERO, DVec3::X).unwrap(), Edge::line(DVec3::X, DVec3::Y).unwrap()];
	assert!(Solid::extrude(&open, DVec3::Z).is_err());
}

#[test]
fn extrude_rejects_a_trailing_edge_after_a_closed_loop() {
	let profile = [plate(4.0), vec![Edge::line(DVec3::Z * 5.0, DVec3::Z * 6.0).unwrap()]].concat();
	assert!(Solid::extrude(&profile, DVec3::Z).is_err(), "a stray edge must not be silently dropped");
}

#[test]
fn extrude_rejects_a_hole_outside_the_outer_loop() {
	let disjoint = [plate(2.0), bore(0.5, 20.0)].concat();
	assert!(Solid::extrude(&disjoint, DVec3::Z).is_err(), "disjoint loops do not bound one face");
}

/// Same square as `plate` but traced the other way round.
fn plate_reversed(side: f64) -> Vec<Edge> {
	let h = side / 2.0;
	Edge::polygon(&[DVec3::new(-h, h, 0.0), DVec3::new(h, h, 0.0), DVec3::new(h, -h, 0.0), DVec3::new(-h, -h, 0.0)]).unwrap()
}

#[test]
fn extrude_hole_winding_does_not_matter() {
	let expected = (100.0 - 4.0) * 2.0;
	for hole in [plate(2.0), plate_reversed(2.0)] {
		let solid = Solid::extrude(&[plate(10.0), hole].concat(), DVec3::Z * 2.0).expect("square hole");
		assert!((solid.volume() - expected).abs() < TOL, "volume={} expected={expected}", solid.volume());
	}
}

#[test]
fn extrude_outer_winding_does_not_matter() {
	let expected = (100.0 - 4.0) * 2.0;
	for outer in [plate(10.0), plate_reversed(10.0)] {
		let solid = Solid::extrude(&[outer, plate(2.0)].concat(), DVec3::Z * 2.0).expect("square hole");
		assert!((solid.volume() - expected).abs() < TOL, "volume={} expected={expected}", solid.volume());
	}
}
