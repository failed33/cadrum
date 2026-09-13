//! Integration tests for `Solid::revolve`, checked against analytic volumes.
//!
//! Profiles lie in the XZ plane and turn about the Z axis, so Pappus applies:
//! a full turn sweeps `2π · x̄ · A` for a profile of area `A` whose centroid
//! sits `x̄` from the axis — and a hole removes its own `2π · x̄ · A`.

use cadrum::{DVec3, Edge, Solid};
use std::f64::consts::{PI, TAU};

fn close(got: f64, want: f64) -> bool {
	(got - want).abs() < 1e-6 * want.abs().max(1.0)
}

/// Rectangle `x ∈ [x0, x1]`, `z ∈ [z0, z1]` in the XZ plane.
fn rect(x0: f64, x1: f64, z0: f64, z1: f64) -> Vec<Edge> {
	Edge::polygon(&[DVec3::new(x0, 0.0, z0), DVec3::new(x1, 0.0, z0), DVec3::new(x1, 0.0, z1), DVec3::new(x0, 0.0, z1)]).unwrap()
}

/// The same rectangle traced the other way round.
fn rect_reversed(x0: f64, x1: f64, z0: f64, z1: f64) -> Vec<Edge> {
	Edge::polygon(&[DVec3::new(x0, 0.0, z1), DVec3::new(x1, 0.0, z1), DVec3::new(x1, 0.0, z0), DVec3::new(x0, 0.0, z0)]).unwrap()
}

#[test]
fn revolve_full_turn_is_a_pipe() {
	let solid = Solid::revolve(&rect(2.0, 5.0, 0.0, 4.0), DVec3::ZERO, DVec3::Z, TAU).expect("pipe");
	let want = PI * (25.0 - 4.0) * 4.0;
	assert!(close(solid.volume(), want), "volume={} want={want}", solid.volume());
}

#[test]
fn revolve_partial_turn_scales_with_the_angle() {
	let full = Solid::revolve(&rect(2.0, 5.0, 0.0, 4.0), DVec3::ZERO, DVec3::Z, TAU).unwrap();
	let quarter = Solid::revolve(&rect(2.0, 5.0, 0.0, 4.0), DVec3::ZERO, DVec3::Z, TAU / 4.0).unwrap();
	assert!(close(quarter.volume(), full.volume() / 4.0), "quarter={} full={}", quarter.volume(), full.volume());
}

#[test]
fn revolve_negative_angle_turns_the_other_way() {
	let forward = Solid::revolve(&rect(2.0, 5.0, 0.0, 4.0), DVec3::ZERO, DVec3::Z, TAU / 4.0).unwrap();
	let backward = Solid::revolve(&rect(2.0, 5.0, 0.0, 4.0), DVec3::ZERO, DVec3::Z, -TAU / 4.0).unwrap();
	assert!(close(backward.volume(), forward.volume()));
	// Same volume, swept to opposite sides of the XZ plane: probe at +-45 degrees.
	let (a, b) = (DVec3::new(2.47, 2.47, 2.0), DVec3::new(2.47, -2.47, 2.0));
	assert!(forward.contains(a) && !forward.contains(b));
	assert!(backward.contains(b) && !backward.contains(a));
}

#[test]
fn revolve_hole_removes_its_pappus_volume() {
	let profile = [rect(1.0, 5.0, 0.0, 6.0), rect(2.5, 3.5, 2.5, 3.5)].concat();
	let solid = Solid::revolve(&profile, DVec3::ZERO, DVec3::Z, TAU).expect("ring with channel");
	let want = TAU * 3.0 * 24.0 - TAU * 3.0 * 1.0;
	assert!(close(solid.volume(), want), "volume={} want={want}", solid.volume());
	assert!(!solid.contains(DVec3::new(3.0, 0.0, 3.0)), "the channel must be hollow");
	assert!(solid.contains(DVec3::new(1.5, 0.0, 1.0)));
}

#[test]
fn revolve_hole_winding_does_not_matter() {
	let want = TAU * 3.0 * 24.0 - TAU * 3.0 * 1.0;
	for hole in [rect(2.5, 3.5, 2.5, 3.5), rect_reversed(2.5, 3.5, 2.5, 3.5)] {
		let solid = Solid::revolve(&[rect(1.0, 5.0, 0.0, 6.0), hole].concat(), DVec3::ZERO, DVec3::Z, TAU).unwrap();
		assert!(close(solid.volume(), want), "volume={} want={want}", solid.volume());
	}
}

#[test]
fn revolve_half_disc_about_its_diameter_is_a_sphere() {
	// The straight edge lies on the axis, so the sweep collapses it to the poles.
	let r = 3.0;
	let profile = [Edge::arc_3pts(DVec3::new(0.0, 0.0, -r), DVec3::new(r, 0.0, 0.0), DVec3::new(0.0, 0.0, r)).unwrap(), Edge::line(DVec3::new(0.0, 0.0, r), DVec3::new(0.0, 0.0, -r)).unwrap()];
	let sphere = Solid::revolve(&profile, DVec3::ZERO, DVec3::Z, TAU).expect("sphere");
	assert!(close(sphere.volume(), 4.0 / 3.0 * PI * r * r * r), "volume={}", sphere.volume());
}

#[test]
fn revolve_rejects_a_zero_axis_or_angle() {
	assert!(Solid::revolve(&rect(2.0, 5.0, 0.0, 4.0), DVec3::ZERO, DVec3::ZERO, TAU).is_err(), "zero axis");
	assert!(Solid::revolve(&rect(2.0, 5.0, 0.0, 4.0), DVec3::ZERO, DVec3::Z, 0.0).is_err(), "zero angle");
	assert!(Solid::revolve(&rect(2.0, 5.0, 0.0, 4.0), DVec3::ZERO, DVec3::Z, TAU * 1.5).is_err(), "beyond a full turn");
}

#[test]
fn revolve_rejects_a_hole_outside_the_outer_loop() {
	let disjoint = [rect(1.0, 2.0, 0.0, 1.0), rect(5.0, 6.0, 5.0, 6.0)].concat();
	assert!(Solid::revolve(&disjoint, DVec3::ZERO, DVec3::Z, TAU).is_err());
}
