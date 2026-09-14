//! Integration tests for `Shell`, the open-surface carrier.
//!
//! Covers one case per entry point: sew (open and closed), offset, fill,
//! free boundaries, and the BRep round trip. `Solid`'s closed guarantee is
//! covered by `tests/solid_sew.rs` and is asserted here only where the two
//! meet — the same five faces that `Shell::sew` accepts, `Solid::sew` refuses.

use cadrum::{Continuity, DVec3, Edge, Error, Face, Filling, JoinType, ShapeKind, Shell, Solid};
use std::f64::consts::PI;

const TOLERANCE: f64 = 1.0e-6;
const SIDE: f64 = 10.0;

/// Five of a cube's six faces: one closed shell short of a solid.
fn open_box(cube: &Solid) -> Shell {
	let faces: Vec<&Face> = cube.iter_face().take(5).collect();
	Shell::sew(faces, TOLERANCE).expect("five faces must sew into an open shell")
}

#[test]
fn test_shell_01_sew_keeps_an_open_face_set_as_a_shell() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(SIDE));
	let shell = open_box(&cube);

	assert_eq!(shell.kind(), ShapeKind::Shell);
	assert_eq!(shell.iter_face().count(), 5);
	assert!(shell.is_valid().expect("validity check"));

	let expected = 5.0 * SIDE * SIDE;
	assert!((shell.area() - expected).abs() / expected < 1.0e-9, "open shell area {} vs {}", shell.area(), expected);

	// The same faces are not a solid, and `Solid` still says so.
	let faces: Vec<&Face> = cube.iter_face().take(5).collect();
	assert!(matches!(Solid::sew(faces, TOLERANCE), Err(Error::Sew(_))));
}

#[test]
fn test_shell_02_sew_keeps_a_closed_face_set_as_a_shell_too() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(SIDE));
	let shell = Shell::sew(cube.iter_face(), TOLERANCE).expect("six faces must sew");

	assert_eq!(shell.kind(), ShapeKind::Shell, "a closed shell is never upgraded to a solid here");
	assert_eq!(shell.iter_face().count(), 6);
	assert!(shell.free_boundaries().expect("free boundaries").is_empty(), "a closed shell has no free boundary");
}

#[test]
fn test_shell_03_sew_rejects_faces_it_could_not_attach() {
	let near = Solid::cube(DVec3::ZERO, DVec3::splat(SIDE));
	let far = Solid::cube(DVec3::splat(100.0), DVec3::splat(100.0 + SIDE));
	let apart: Vec<&Face> = near.iter_face().take(1).chain(far.iter_face().take(1)).collect();

	let error = Shell::sew(apart, TOLERANCE).expect_err("disjoint faces must not sew into one shell");
	assert!(matches!(error, Error::Sew(_)), "expected Error::Sew, got {error:?}");
}

#[test]
fn test_shell_04_offset_grows_an_open_shell() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(SIDE));
	let shell = open_box(&cube);
	let distance = 1.0;

	let grown = shell.offset(distance, TOLERANCE, JoinType::Intersection).expect("outward shell offset");

	assert_eq!(grown.kind(), ShapeKind::Shell);
	assert!(grown.is_valid().expect("validity check"));
	// Five faces of a box grown by `distance` with sharp corners: the four
	// sides gain `distance` in each open direction, the bottom in two.
	let expected = 4.0 * (SIDE + distance) * (SIDE + 2.0 * distance) + (SIDE + 2.0 * distance).powi(2);
	assert!((grown.area() - expected).abs() / expected < 1.0e-6, "offset shell area {} vs {}", grown.area(), expected);
}

#[test]
fn test_shell_05_fill_closes_a_boundary_loop() {
	let radius = 4.0;
	let circle = Edge::circle(radius, DVec3::Z).expect("circle");

	let disc = Shell::fill(std::slice::from_ref(&circle), Filling { continuity: Continuity::C0, ..Filling::default() }).expect("filling a closed loop");

	assert_eq!(disc.kind(), ShapeKind::Shell);
	assert_eq!(disc.iter_face().count(), 1);
	let expected = PI * radius * radius;
	assert!((disc.area() - expected).abs() / expected < 1.0e-2, "filled disc area {} vs πr² {}", disc.area(), expected);
}

#[test]
fn test_shell_06_fill_rejects_an_empty_boundary() {
	let none: Vec<&Edge> = Vec::new();
	let error = Shell::fill(none, Filling::default()).expect_err("an empty boundary must be refused");
	assert!(matches!(error, Error::Surface(_)), "expected Error::Surface, got {error:?}");
}

#[test]
fn test_shell_07_free_boundaries_report_the_missing_face_as_one_loop() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(SIDE));
	let shell = open_box(&cube);

	let loops = shell.free_boundaries().expect("free boundaries");

	assert_eq!(loops.len(), 1, "one missing face leaves one boundary loop");
	assert_eq!(loops[0].len(), 4, "the loop is the four edges of the missing face");
	let perimeter: f64 = loops[0].iter().map(|edge| (edge.end_point() - edge.start_point()).length()).sum();
	assert!((perimeter - 4.0 * SIDE).abs() < 1.0e-6, "loop perimeter {perimeter} vs {}", 4.0 * SIDE);
}

#[test]
fn test_shell_08_brep_round_trip_preserves_the_open_shell() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(SIDE));
	let shell = open_box(&cube);

	let mut buf = Vec::new();
	Shell::write_brep(std::slice::from_ref(&shell), &mut buf).expect("write_brep");
	let decoded = Shell::read_brep(&mut buf.as_slice()).expect("read_brep");

	assert_eq!(decoded.len(), 1, "one shell written, one read back");
	assert_eq!(decoded[0].kind(), ShapeKind::Shell);
	assert_eq!(decoded[0].iter_face().count(), 5);
	assert!((decoded[0].area() - shell.area()).abs() < 1.0e-9);
	assert_eq!(decoded[0].free_boundaries().expect("free boundaries").len(), 1);
	// The reader is not filtered to `TopAbs_SOLID`, which is what a solid-only
	// reader would have dropped the payload for.
	assert!(Solid::read_brep(&mut buf.as_slice()).expect("solid read").is_empty());
}

#[test]
fn test_shell_09_mesh_attaches_a_face_to_every_triangle() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(SIDE));
	let shell = open_box(&cube);

	let mesh = Shell::mesh(std::slice::from_ref(&shell), Default::default()).expect("mesh");

	assert_eq!(mesh.face_ids.len(), mesh.indices.len() / 3);
	let faces: Vec<u64> = shell.iter_face().map(Face::id).collect();
	assert!(mesh.face_ids.iter().all(|id| faces.contains(id)), "a triangle referenced a face outside the shell");
}

#[test]
fn test_shell_10_fill_rejects_a_boundary_enclosing_no_area() {
	let far = DVec3::new(SIDE, 0.0, 0.0);
	let there = Edge::line(DVec3::ZERO, far).expect("line");
	let back = Edge::line(far, DVec3::ZERO).expect("line");

	// A loop that doubles back on itself bounds no surface. OCCT's filling
	// faults on it instead of raising, so without the binding's
	// signal-to-exception translation this call aborts the process.
	let error = Shell::fill([&there, &back], Filling { continuity: Continuity::C0, ..Filling::default() }).expect_err("a boundary enclosing no area must be refused");
	assert!(matches!(error, Error::Surface(_)), "expected Error::Surface, got {error:?}");
}
