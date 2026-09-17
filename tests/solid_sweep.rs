//! Integration tests for `Solid::sweep`.
//!
//! Covers:
//! - 閉じた periodic spine + periodic auxiliary guide の sweep が通る
//! - 捻れた輪の体積が Pappus の定理と一致する

use cadrum::{BSplineEnd, DQuat, DVec3, Edge, Error, GuideCorrespondence, ProfileOrient, Solid};

/// spine/aux の補間点数。
const CURVE_STEPS: usize = 24;
/// spine の半径と profile の一辺。
const RADIUS: f64 = 10.;
const SIDE: f64 = 2.;

/// 半径 RADIUS の円上の点と、そこから接線まわりに theta 捻った guide 点。
fn wire_and_guide(phi: f64) -> [DVec3; 2] {
	let theta = (phi * 2.).sin();
	let rot_z = DQuat::from_rotation_z(phi);
	let rot_y = DQuat::from_rotation_y(theta);
	let wire_point = DVec3::X * RADIUS;
	let guide_offset = DVec3::X;
	[rot_z * wire_point, rot_z * (wire_point + rot_y * guide_offset)]
}

/// [0, 2π) を n 等分した点列。periodic B-spline へ渡すので始点は繰り返さない。
fn sample(idx: usize, n: usize) -> Vec<DVec3> {
	(0..n).map(|i| wire_and_guide(i as f64 / n as f64 * std::f64::consts::TAU)[idx]).collect()
}

/// 一辺 SIDE の正方形を spine 始点の接線に直交させ、始点へ移動。
fn square_profile(spine: &Edge) -> Result<Vec<Edge>, Error> {
	let h = SIDE / 2.;
	let square = Edge::polygon(&[DVec3::new(-h, -h, 0.), DVec3::new(h, -h, 0.), DVec3::new(h, h, 0.), DVec3::new(-h, h, 0.)])?;
	Ok(square.into_iter().map(|e| e.align_z(spine.start_tangent(), spine.start_point()).translate(spine.start_point())).collect())
}

/// spine も aux も periodic B-spline 1 本のまま `ProfileOrient::Auxiliary` で sweep。
/// 素の OCCT 8.0.1 ではこの組み合わせは Error::Sweep になるか破綻形状を返すので、
/// patches/pr1.patch (fix D) と patches/pr2.patch (fix A) が効いていることの回帰テスト。
fn closed_auxiliary_sweep(correspondence: GuideCorrespondence) -> Result<Solid, Error> {
	let spine = Edge::bspline(&sample(0, CURVE_STEPS), BSplineEnd::Periodic)?;
	let aux = Edge::bspline(&sample(1, CURVE_STEPS), BSplineEnd::Periodic)?;
	let profile = square_profile(&spine)?;
	Solid::sweep(&profile, &[spine], ProfileOrient::Auxiliary { guide: &[aux], correspondence })
}

// ==================== (1) 閉じた spine + guide の sweep が通る ====================

#[test]
fn test_sweep_01_closed_periodic_auxiliary_succeeds() {
	let solid = closed_auxiliary_sweep(GuideCorrespondence::NormalPlane).expect("closed periodic spine with a periodic guide must sweep");

	assert_eq!(solid.iter_face().count(), 4, "a square profile swept along a closed spine leaves 4 side faces and no cap");

	let mesh = Solid::mesh(std::iter::once(&solid), cadrum::Tessellation { deflection_linear: 0.01, relative_linear: false, ..Default::default() }).expect("mesh should succeed");
	assert!(!mesh.vertices.is_empty(), "swept solid must tessellate");
}

// ==================== (2) 体積が Pappus の定理と一致 ====================

#[test]
fn test_sweep_02_closed_auxiliary_volume_matches_pappus() {
	let solid = closed_auxiliary_sweep(GuideCorrespondence::NormalPlane).expect("closed periodic spine with a periodic guide must sweep");

	// 断面積 SIDE² の輪。捻っても重心は半径 RADIUS の円上なので体積は面積×周長。
	// The periodic spline approximates the circle; its length differs by 6.4 ppm.
	// Normal-plane correspondence keeps the square perpendicular to the spine.
	let expected = SIDE * SIDE * std::f64::consts::TAU * RADIUS;
	let rel = (solid.volume().expect("volume integration") - expected).abs() / expected;
	assert!(rel < 1.0e-5, "volume {:.3} vs Pappus {:.3} (relative error {:.3e})", solid.volume().expect("volume integration"), expected, rel);
}

#[test]
fn arc_length_guidance_preserves_its_tilt_instead_of_using_the_normal_plane() {
	use cadrum::{apply, Algorithm, Frame, Shape};
	let length = 10.0;
	let offset = 4.0;
	let rise = 3.0;
	let spine = Edge::line(DVec3::ZERO, DVec3::Z * length).expect("spine");
	let guide = Edge::line(DVec3::X * offset, DVec3::new(offset, 0.0, length + rise)).expect("guide");
	let profile = Edge::polygon(&[DVec3::new(-1.0, -1.0, 0.0), DVec3::new(1.0, -1.0, 0.0), DVec3::new(1.0, 1.0, 0.0), DVec3::new(-1.0, 1.0, 0.0)]).expect("square");
	let wire = |edges: &[Edge]| -> Shape { apply(Algorithm::Wire { edges: &edges.iter().collect::<Vec<_>>() }).expect("wire").shape };
	let split = [Edge::line(DVec3::ZERO, DVec3::Z * (length * 0.4)).expect("first span"), Edge::line(DVec3::Z * (length * 0.4), DVec3::Z * length).expect("second span")];
	let spines = [wire(&[spine]), wire(&split)];
	let guide = wire(&[guide]);
	let profile = wire(&profile);
	for spine in &spines {
		for correspondence in [GuideCorrespondence::ArcLength, GuideCorrespondence::NormalPlane] {
			let shape = apply(Algorithm::PipeShell { spine, sections: &[&profile], frame: Frame::Auxiliary { guide: &guide, correspondence }, law: &[], tolerance: Some(1.0e-6), solid: true }).expect("guided sweep within construction accuracy").shape;
			// Integrate section area times the projection onto the spine tangent.
			let expected = 4.0
				* length * match correspondence {
				GuideCorrespondence::ArcLength => offset / rise * (rise / offset).asinh(),
				GuideCorrespondence::NormalPlane => 1.0,
			};
			let actual = shape.volume().expect("volume");
			assert!((actual / expected - 1.0).abs() < 1.0e-6, "{correspondence:?}: {actual} vs {expected}");
		}
	}
	// A spline guide has varying parameter speed; splitting the spine must
	// preserve the guide's arc-length correspondence and its second derivative.
	let curved_guide = Edge::bspline(&[DVec3::new(4.0, 0.0, 0.0), DVec3::new(4.5, 1.0, 2.0), DVec3::new(5.0, -0.5, 7.0), DVec3::new(4.0, 0.0, 13.0)], BSplineEnd::NotAKnot).expect("curved guide");
	let curved_guide = wire(&[curved_guide]);
	let volumes = spines.each_ref().map(|spine| apply(Algorithm::PipeShell { spine, sections: &[&profile], frame: Frame::Auxiliary { guide: &curved_guide, correspondence: GuideCorrespondence::ArcLength }, law: &[], tolerance: Some(1.0e-6), solid: true }).expect("curved guide within construction accuracy").shape.volume().expect("volume"));
	assert!((volumes[0] / volumes[1] - 1.0).abs() < 1.0e-6, "splitting the spine changed volume: {volumes:?}");
}

#[test]
fn closed_arc_length_sweep_converges_and_meshes_at_fine_absolute_deflection() {
	let solid = closed_auxiliary_sweep(GuideCorrespondence::ArcLength).expect("arc-length sweep");
	let mesh = Solid::mesh([&solid], cadrum::Tessellation { deflection_linear: 0.005, relative_linear: false, ..Default::default() }).expect("fine arc-length tessellation");
	assert!(!mesh.indices.is_empty());
	let mesh_volume = mesh.indices.chunks_exact(3).map(|triangle| mesh.vertices[triangle[0]].dot(mesh.vertices[triangle[1]].cross(mesh.vertices[triangle[2]])) / 6.0).sum::<f64>().abs();
	let volume = solid.volume().expect("volume");
	assert!((mesh_volume / volume - 1.0).abs() < 1e-3, "bounded mesh reference {mesh_volume} vs {volume}");
}
