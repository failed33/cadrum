//! The algorithm table, row by row, on the kind-generic carrier. Rows that
//! `Solid` already exercises through its own suite (primitives, prism, sweep,
//! loft, thick solid, offset faces, cells, fillet, chamfer) are covered there;
//! the rows below are the ones only reachable through `apply`, plus the open
//! shell fixtures wave 7a stated on `Shell`.

use cadrum::{apply, Algorithm, Bevel, BooleanOperation, Continuity, DVec3, Edge, Face, Filling, Frame, JoinType, LawSample, Shape, ShapeKind, Solid};
use std::f64::consts::PI;

const TOLERANCE: f64 = 1.0e-6;
const SIDE: f64 = 10.0;

fn cube() -> Solid {
	Solid::cube(DVec3::ZERO, DVec3::splat(SIDE))
}

fn shape(applied: Result<cadrum::Applied, cadrum::Error>) -> Shape {
	applied.expect("the row must apply").shape
}

fn wire(edges: &[Edge]) -> Shape {
	let edges: Vec<&Edge> = edges.iter().collect();
	shape(apply(Algorithm::Wire { edges: &edges }))
}

/// Five of a cube's six faces, one face short of a closed shell.
fn open_box() -> Shape {
	let cube = cube();
	let faces: Vec<&Face> = cube.iter_face().take(5).collect();
	let sewn = shape(apply(Algorithm::Sew { faces: &faces, tolerance: TOLERANCE }));
	let mut shells = sewn.components(ShapeKind::Shell);
	assert_eq!(shells.len(), 1, "five adjoining faces sew into one shell");
	shells.pop().expect("one shell")
}

#[test]
fn sewing_five_faces_keeps_an_open_shell_that_a_solid_refuses() {
	let shell = open_box();
	assert_eq!(shell.kind(), ShapeKind::Shell);
	assert!(!shell.is_closed());
	assert_eq!(shell.iter_face().count(), 5);
	assert!(shell.is_valid().expect("validity check"));
	let expected = 5.0 * SIDE * SIDE;
	assert!((shell.area().expect("area integration") - expected).abs() / expected < 1.0e-9, "open shell area {} vs {}", shell.area().expect("area integration"), expected);
	let loops = shell.free_boundaries().expect("free boundaries");
	assert_eq!(loops.len(), 1, "one missing face leaves one loop");
	assert_eq!(loops[0].len(), 4);

	let cube = cube();
	let faces: Vec<&Face> = cube.iter_face().take(5).collect();
	assert!(matches!(Solid::sew(faces, TOLERANCE), Err(cadrum::Error::Sew(_))));
}

#[test]
fn sewing_six_faces_closes_the_shell_without_making_it_a_solid() {
	let cube = cube();
	let faces: Vec<&Face> = cube.iter_face().collect();
	let sewn = shape(apply(Algorithm::Sew { faces: &faces, tolerance: TOLERANCE }));
	assert_eq!(sewn.kind(), ShapeKind::Shell, "a closed shell is never upgraded here");
	assert!(sewn.is_closed());
	assert!(sewn.free_boundaries().expect("free boundaries").is_empty());
}

#[test]
fn faces_that_do_not_meet_stay_apart_after_sewing() {
	let near = cube();
	let far = cube().translate(DVec3::X * 100.0);
	let apart: Vec<&Face> = near.iter_face().take(1).chain(far.iter_face().take(1)).collect();
	let sewn = shape(apply(Algorithm::Sew { faces: &apart, tolerance: TOLERANCE }));
	assert!(sewn.components(ShapeKind::Shell).is_empty(), "two disjoint faces form no shell");
	assert_eq!(sewn.components(ShapeKind::Face).len(), 2);
}

#[test]
fn an_open_shell_offsets_outward_and_stays_a_shell() {
	let shell = open_box();
	let distance = 1.0;
	let grown = shape(apply(Algorithm::OffsetShape { shape: &shell, offset: distance, tolerance: TOLERANCE, join: JoinType::Intersection, intersection: true }));
	let grown = grown.components(ShapeKind::Shell).pop().expect("one shell");
	assert!(grown.is_valid().expect("validity check"));
	let expected = 4.0 * (SIDE + distance) * (SIDE + 2.0 * distance) + (SIDE + 2.0 * distance).powi(2);
	assert!((grown.area().expect("area integration") - expected).abs() / expected < 1.0e-6, "offset shell area {} vs {}", grown.area().expect("area integration"), expected);
}

#[test]
fn an_offset_box_maps_every_face_onto_its_source() {
	let block = cube();
	let applied = apply(Algorithm::OffsetShape { shape: block.as_shape(), offset: 1.0, tolerance: TOLERANCE, join: JoinType::Intersection, intersection: true }).expect("offset");
	let faces: std::collections::HashSet<u64> = block.iter_face().map(Face::id).collect();
	let imaged: std::collections::HashSet<u64> = applied.history.iter().filter(|[_, source]| faces.contains(source)).map(|[image, _]| *image).collect();
	for face in applied.shape.iter_face() {
		assert!(imaged.contains(&face.id()), "offset face {} descends from no face of the box", face.id());
	}
	let sources: std::collections::HashSet<u64> = applied.history.iter().map(|[_, source]| *source).collect();
	assert!(faces.iter().all(|face| sources.contains(face)), "every face of the box has an offset image");
}

#[test]
fn filling_a_closed_loop_yields_one_face() {
	let circle = Edge::circle(3.0, DVec3::Z).expect("circle");
	let disc = shape(apply(Algorithm::Filling { boundary: &[&circle], filling: Filling { continuity: Continuity::C0, ..Filling::default() } }));
	assert_eq!(disc.iter_face().count(), 1);
	let expected = PI * 9.0;
	assert!((disc.area().expect("area integration") - expected).abs() / expected < 1.0e-2, "filled disc area {} vs {}", disc.area().expect("area integration"), expected);
	assert!(apply(Algorithm::Filling { boundary: &[], filling: Filling::default() }).is_err(), "an empty boundary must be refused");
}

#[test]
fn a_boundary_enclosing_no_area_is_refused_rather_than_aborting() {
	for direction in [DVec3::X, DVec3::new(1.0, 2.0, 3.0).normalize()] {
		let start = DVec3::new(7.0, -3.0, 2.0);
		let there = Edge::line(start, start + direction * SIDE).expect("line");
		let back = Edge::line(start + direction * SIDE, start).expect("line");
		let error = apply(Algorithm::Filling { boundary: &[&there, &back], filling: Filling::default() }).expect_err("collinear boundary");
		assert!(matches!(error, cadrum::Error::Algorithm(reason) if reason.contains("does not span a surface")));
	}
}

#[test]
fn a_shell_round_trips_through_brep_and_meshes_with_face_ancestry() {
	let shell = open_box();
	let mut buffer = Vec::new();
	Shape::write_brep([&shell], &mut buffer).expect("write_brep");
	let decoded = Shape::read_brep(&mut buffer.as_slice()).expect("read_brep");
	let decoded = decoded.components(ShapeKind::Shell).pop().expect("one shell");
	assert_eq!(decoded.iter_face().count(), 5);
	assert!((decoded.area().expect("area integration") - shell.area().expect("area integration")).abs() < 1.0e-9);
	assert_eq!(decoded.free_boundaries().expect("free boundaries").len(), 1);

	let mesh = Shape::mesh([&shell], Default::default()).expect("mesh");
	let faces: Vec<u64> = shell.iter_face().map(Face::id).collect();
	assert!(!mesh.indices.is_empty());
	assert!(mesh.face_ids.iter().all(|id| faces.contains(id)));
	assert!(faces.iter().all(|id| mesh.face_ids.contains(id)));
}

#[test]
fn a_revolution_sweeps_a_profile_about_an_axis() {
	let square = Edge::polygon(&[DVec3::new(2.0, 0.0, 0.0), DVec3::new(4.0, 0.0, 0.0), DVec3::new(4.0, 0.0, 1.0), DVec3::new(2.0, 0.0, 1.0)]).expect("square");
	let face = shape(apply(Algorithm::Face { wire: &wire(&square) }));
	let ring = shape(apply(Algorithm::Revolution { base: &face, axis_origin: DVec3::ZERO, axis_direction: DVec3::Z, angle: 2.0 * PI }));
	assert_eq!(ring.kind(), ShapeKind::Solid);
	let expected = PI * (16.0 - 4.0);
	assert!((ring.volume().expect("volume integration") - expected).abs() / expected < 1.0e-6, "ring volume {} vs {}", ring.volume().expect("volume integration"), expected);
}

#[test]
fn a_scale_law_sweep_reports_its_two_ends() {
	let spine = wire(&[Edge::line(DVec3::ZERO, DVec3::Z * 30.0).expect("line")]);
	let section = wire(&[Edge::circle(1.0, DVec3::Z).expect("circle")]);
	let law = [LawSample { station: 0.0, value: 2.0 }, LawSample { station: 1.0, value: 4.0 }];
	let applied = apply(Algorithm::PipeShell { spine: &spine, sections: &[&section], frame: Frame::CorrectedFrenet, law: &law, tolerance: Some(TOLERANCE), solid: true }).expect("law sweep");
	let expected = PI * 30.0 / 3.0 * (4.0 + 8.0 + 16.0);
	assert!((applied.shape.volume().expect("volume integration") / expected - 1.0).abs() < 1.0e-6);
	let [start, end] = applied.ends.expect("a sweep publishes its ends");
	let rims: Vec<u64> = applied.shape.iter_edge().map(Edge::id).collect();
	for end in [&start, &end] {
		assert_eq!(end.iter_edge().count(), 1);
		assert!(end.iter_edge().all(|edge| rims.contains(&edge.id())), "an end edge is an edge of the solid");
	}
	assert_ne!(start.iter_edge().next().map(Edge::id), end.iter_edge().next().map(Edge::id));
}

#[test]
fn booleans_fuse_cut_and_common_two_blocks() {
	let a = cube();
	let b = cube().translate(DVec3::X * SIDE / 2.0);
	let (a, b) = (a.as_shape(), b.as_shape());
	for (operation, expected) in [(BooleanOperation::Union, 1500.0), (BooleanOperation::Difference, 500.0), (BooleanOperation::Intersection, 500.0)] {
		let expression = cadrum::Expression::from(a).combine(operation, cadrum::Expression::from(b));
		let applied = apply(Algorithm::Boolean { expression: &expression }).expect("boolean");
		let volume: f64 = applied.shape.components(ShapeKind::Solid).iter().map(Shape::volume).sum::<Result<f64, _>>().expect("volume integration");
		assert!((volume - expected).abs() < 1.0e-6, "{operation:?}: {volume} vs {expected}");
		assert!(!applied.history.is_empty(), "{operation:?} publishes face history");
	}
}

#[test]
fn a_splitter_keeps_both_sides_and_a_section_yields_the_cut_curve() {
	let block = cube();
	let knife = Solid::half_space(DVec3::splat(SIDE / 2.0), DVec3::Z);
	let split = shape(apply(Algorithm::Splitter { arguments: &[block.as_shape()], tools: &[knife.as_shape()] }));
	let halves = split.components(ShapeKind::Solid);
	assert_eq!(halves.len(), 2);
	assert!(halves.iter().all(|half| (half.volume().expect("volume integration") - 500.0).abs() < 1.0e-6));

	let section = shape(apply(Algorithm::Section { arguments: &[block.as_shape()], tools: &[knife.as_shape()] }));
	assert_eq!(section.components(ShapeKind::Edge).len(), 4, "a plane through a cube cuts a square");
}

#[test]
fn an_affine_transform_maps_every_face_onto_its_image() {
	let block = cube();
	let matrix = [2.0, 0.0, 0.0, 5.0, 0.0, 3.0, 0.0, 7.0, 0.0, 0.0, 4.0, 11.0];
	let applied = apply(Algorithm::Transform { shape: block.as_shape(), matrix }).expect("transform");
	assert!((applied.shape.volume().expect("volume integration") - 24.0 * SIDE.powi(3)).abs() < 1.0e-6);
	let sources: std::collections::HashSet<u64> = applied.history.iter().map(|[_, source]| *source).collect();
	assert!(block.iter_face().all(|face| sources.contains(&face.id())), "every face has an image");
	assert!(block.iter_edge().all(|edge| sources.contains(&edge.id())), "every edge has an image");
}

#[test]
fn a_draft_angle_tapers_the_side_faces() {
	let block = cube();
	let sides: Vec<&Face> = block.iter_face().filter(|face| face.iter_edge().any(|edge| (edge.start_point().z - edge.end_point().z).abs() > 1.0e-9)).collect();
	assert_eq!(sides.len(), 4);
	let drafted = shape(apply(Algorithm::DraftAngle { shape: block.as_shape(), faces: &sides, direction: DVec3::Z, angle: 5.0_f64.to_radians(), plane_origin: DVec3::ZERO, plane_normal: DVec3::Z }));
	assert_eq!(drafted.kind(), ShapeKind::Solid);
	assert!(drafted.is_valid().expect("validity check"));
	assert!((drafted.volume().expect("volume integration") - block.volume().expect("volume integration")).abs() > 1.0, "a five degree draft on four sides moves material");
}

#[test]
fn a_projection_lays_a_wire_onto_a_shape() {
	let block = cube();
	let square = Edge::polygon(&[DVec3::new(2.0, 2.0, 20.0), DVec3::new(8.0, 2.0, 20.0), DVec3::new(8.0, 8.0, 20.0), DVec3::new(2.0, 8.0, 20.0)]).expect("square");
	let projected = shape(apply(Algorithm::Projection { wire: &wire(&square), onto: block.as_shape(), direction: DVec3::Z }));
	let edges = projected.components(ShapeKind::Edge);
	assert!(!edges.is_empty());
	assert!(
		edges.iter().all(|edge| {
			let [low, high] = edge.bounding_box();
			(low.z - SIDE).abs() < 1.0e-6 && (high.z - SIDE).abs() < 1.0e-6 || low.z.abs() < 1.0e-6 && high.z.abs() < 1.0e-6
		}),
		"the projection lands on the top and bottom faces"
	);
}

#[test]
fn unifying_a_fused_pair_merges_coplanar_faces() {
	let a = cube();
	let b = cube().translate(DVec3::X * SIDE);
	let expression = cadrum::Expression::from(a.as_shape()).combine(BooleanOperation::Union, cadrum::Expression::from(b.as_shape()));
	let fused = shape(apply(Algorithm::Boolean { expression: &expression }));
	let unified = apply(Algorithm::Unify { shape: &fused }).expect("unify");
	assert_eq!(unified.shape.iter_face().count(), 6, "two blocks in a row are one box");
	assert!(!unified.history.is_empty());
}

/// The cube's upright edge at the origin corner, with the two faces it
/// separates in the order the solid enumerates them.
fn upright_corner(block: &Solid) -> (&Edge, Vec<&Face>) {
	let edge = block
		.iter_edge()
		.find(|edge| {
			let (start, end) = (edge.start_point(), edge.end_point());
			[start, end].iter().all(|point| point.x.abs() < TOLERANCE && point.y.abs() < TOLERANCE) && (start.z - end.z).abs() > 1.0
		})
		.expect("the cube has an upright edge at the origin");
	let faces: Vec<&Face> = block.iter_face().filter(|face| face.iter_edge().any(|other| other.id() == edge.id())).collect();
	assert_eq!(faces.len(), 2, "an edge of a solid separates two faces");
	(edge, faces)
}

#[test]
fn a_radius_law_interpolates_the_fillet_between_its_stations() {
	let block = cube();
	let (edge, _) = upright_corner(&block);
	let rounded = |radius: f64, law: &[LawSample]| shape(apply(Algorithm::Fillet { shape: block.as_shape(), edges: &[edge], radius, law })).volume().expect("volume integration");
	let varied = rounded(1.0, &[LawSample { station: 0.0, value: 1.0 }, LawSample { station: 1.0, value: 3.0 }]);
	let (narrow, wide) = (rounded(1.0, &[]), rounded(3.0, &[]));
	assert!(wide < varied && varied < narrow, "a 1 to 3 law removes more than a constant 1 and less than a constant 3: {narrow} {varied} {wide}");
}

#[test]
fn a_chamfer_by_two_distances_lays_the_first_on_its_reference_face() {
	let block = cube();
	let (edge, faces) = upright_corner(&block);
	let bevelled = |reference: &Face| shape(apply(Algorithm::Chamfer { shape: block.as_shape(), edges: &[edge], references: &[reference], bevel: Bevel::Distances { distance: 1.0, second: 3.0 } }));
	let expected = SIDE.powi(3) - 0.5 * 1.0 * 3.0 * SIDE;
	// The wedge has the same section either way round; where it lies does not.
	// The probe is inside the one measured 3 along the reference's neighbour.
	let probe = DVec3::new(1.5, 0.2, SIDE / 2.0);
	let kept: Vec<bool> = faces
		.iter()
		.map(|reference| {
			let cut = bevelled(reference);
			assert!((cut.volume().expect("volume integration") - expected).abs() < 1.0e-6, "chamfer volume {} vs {expected}", cut.volume().expect("volume integration"));
			cut.contains(probe)
		})
		.collect();
	assert_ne!(kept[0], kept[1], "the reference face decides which side the long distance is measured on");
}

#[test]
fn a_chamfer_by_an_angle_rises_from_its_reference_face() {
	let block = cube();
	let (edge, faces) = upright_corner(&block);
	let (distance, angle) = (2.0, 30.0_f64.to_radians());
	let bevelled = shape(apply(Algorithm::Chamfer { shape: block.as_shape(), edges: &[edge], references: &[faces[0]], bevel: Bevel::Angle { distance, angle } }));
	// The far leg is the near one turned through the angle, so the wedge's
	// section is half the product of the two.
	let expected = SIDE.powi(3) - 0.5 * distance * distance * angle.tan() * SIDE;
	assert!((bevelled.volume().expect("volume integration") - expected).abs() < 1.0e-6, "chamfer volume {} vs {expected}", bevelled.volume().expect("volume integration"));
}

#[test]
fn defeaturing_removes_a_blend_and_heals_the_edge_it_rounded() {
	let block = cube();
	let (edge, _) = upright_corner(&block);
	let rounded = apply(Algorithm::Fillet { shape: block.as_shape(), edges: &[edge], radius: 2.0, law: &[] }).expect("fillet");
	let images: std::collections::HashSet<u64> = rounded.history.iter().map(|[image, _]| *image).collect();
	let blend: Vec<&Face> = rounded.shape.iter_face().filter(|face| !images.contains(&face.id())).collect();
	assert_eq!(blend.len(), 1, "one rounded edge leaves one face descending from none");
	let healed = shape(apply(Algorithm::Defeaturing { shape: &rounded.shape, faces: &blend }));
	assert_eq!(healed.iter_face().count(), 6, "the blend is gone and a box is a box again");
	assert!((healed.volume().expect("volume integration") - block.volume().expect("volume integration")).abs() < 1.0e-6, "healed volume {} vs {}", healed.volume().expect("volume integration"), block.volume().expect("volume integration"));
}
