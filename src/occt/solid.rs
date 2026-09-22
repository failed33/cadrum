//! `Solid`: the closed, positive-volume guarantee over the kind-generic
//! carrier. Every operation is a row of the algorithm table; this type adds
//! the kind check on what the table returned, the face colormap and the
//! history of the last operation, and nothing else.

use super::algorithm::{apply, Algorithm, Applied, Bevel, Frame, JoinType};
use super::edge::Edge;
use super::face::Face;
use super::shape::{Shape, ShapeKind};
use crate::common::boolean::Boolean;
use crate::common::error::Error;
use crate::traits::{ProfileOrient, SolidStruct, Transform};
use glam::DVec3;
use std::sync::Arc;

/// A single solid: a [`Shape`] whose kind is `TopAbs_SOLID`.
pub struct Solid {
	shape: Shape,
	/// Keyed by a face's TShape id, or by `Solid::id()` for the solid as a whole; a face
	/// colour wins over the solid's. Other solids' keys may be present (`decompose`).
	#[cfg(feature = "color")]
	colormap: std::collections::HashMap<u64, crate::common::color::Color>,
	/// Face-derivation history from the most recent operation, as
	/// `[post_id, src_id]` pairs: identity for an untouched face, the
	/// descendant for a modified one. Empty for constructors and I/O reads,
	/// and after a rebuild (scale/mirror/Clone) that leaves no descendant to
	/// name; preserved across translate/rotate/color.
	history: Arc<[[u64; 2]]>,
}

impl Solid {
	/// Wrap a carrier that holds a solid. A null carrier is tolerated for
	/// the sake of `is_null`.
	pub(crate) fn new(shape: Shape, #[cfg(feature = "color")] colormap: std::collections::HashMap<u64, crate::common::color::Color>, history: Arc<[[u64; 2]]>) -> Self {
		debug_assert!(matches!(shape.kind(), ShapeKind::Null | ShapeKind::Solid), "Solid::new called with a non-SOLID shape");
		Solid {
			shape,
			#[cfg(feature = "color")]
			colormap,
			history,
		}
	}

	/// A built constructor result: no colour, no history.
	fn built(shape: Shape) -> Self {
		Self::new(
			shape,
			#[cfg(feature = "color")]
			Default::default(),
			Default::default(),
		)
	}

	/// The one solid a row returned, or the one solid it wrapped in a
	/// compound (fillets and chamfers do), refused otherwise.
	fn single(applied: Applied, refuse: impl FnOnce(String) -> Error) -> Result<(Shape, Vec<[u64; 2]>), Error> {
		let history = applied.history();
		let shape = match applied.shape.kind() {
			ShapeKind::Solid => applied.shape,
			_ => {
				let mut solids = applied.shape.components(ShapeKind::Solid);
				match (solids.pop(), solids.is_empty()) {
					(Some(solid), true) => solid,
					_ => return Err(refuse(format!("expected one solid, got a {:?}", applied.shape.kind()))),
				}
			}
		};
		Ok((shape, history))
	}

	/// A row applied to this solid, carrying colours across its history.
	fn derived(&self, algorithm: Algorithm<'_>, refuse: impl Fn(String) -> Error) -> Result<Self, Error> {
		let (shape, history) = Self::single(apply(algorithm).map_err(|error| refuse(error.to_string()))?, &refuse)?;
		#[cfg(feature = "color")]
		let colormap = self.remap_colormap(&shape, &history);
		Ok(Self::new(
			shape,
			#[cfg(feature = "color")]
			colormap,
			history.into(),
		))
	}

	/// A wire of edges, in the order given.
	fn wire<'a>(edges: impl IntoIterator<Item = &'a Edge>, refuse: impl FnOnce(String) -> Error) -> Result<Shape, Error> {
		let edges: Vec<&Edge> = edges.into_iter().collect();
		if edges.is_empty() {
			return Err(refuse("a section has no edges".into()));
		}
		apply(Algorithm::Wire { edges: &edges }).map(|applied| applied.shape).map_err(|error| refuse(error.to_string()))
	}

	/// The carrier under the guarantee, for rows that take it as an input.
	pub fn as_shape(&self) -> &Shape {
		&self.shape
	}

	// ==================== Color accessors ====================

	/// Read-only access to the per-face colormap.
	#[cfg(feature = "color")]
	pub fn colormap(&self) -> &std::collections::HashMap<u64, crate::common::color::Color> {
		&self.colormap
	}

	/// Mutable access to the per-face colormap.
	#[cfg(feature = "color")]
	pub fn colormap_mut(&mut self) -> &mut std::collections::HashMap<u64, crate::common::color::Color> {
		&mut self.colormap
	}

	/// Carry face colours across `history` pairs, and the solid's own colour
	/// onto the new solid.
	#[cfg(feature = "color")]
	fn remap_colormap(&self, new_shape: &Shape, history: &[[u64; 2]]) -> std::collections::HashMap<u64, crate::common::color::Color> {
		let mut colormap: std::collections::HashMap<u64, crate::common::color::Color> = history.iter().filter_map(|[post, source]| Some((*post, *self.colormap.get(source)?))).collect();
		if let Some(&color) = self.colormap.get(&self.id()) {
			colormap.insert(new_shape.id(), color);
		}
		colormap
	}

	/// Carry colours by face position: what a rebuild that publishes no
	/// history (a copy, a mirror) leaves to go on.
	#[cfg(feature = "color")]
	fn remap_colormap_by_order(&self, new_shape: &Shape) -> std::collections::HashMap<u64, crate::common::color::Color> {
		let mut colormap = std::collections::HashMap::new();
		for (old_face, new_face) in self.shape.iter_face().zip(new_shape.iter_face()) {
			if let Some(&color) = self.colormap.get(&old_face.id()) {
				colormap.insert(new_face.id(), color);
			}
		}
		if let Some(&color) = self.colormap.get(&self.id()) {
			colormap.insert(new_shape.id(), color);
		}
		colormap
	}

	/// What OCCT's `BRepCheck_Analyzer` reports.
	pub fn is_valid(&self) -> Result<bool, Error> {
		self.shape.is_valid()
	}

	/// Returns `true` if this solid wraps a null shape.
	pub fn is_null(&self) -> bool {
		self.shape.is_null()
	}
}

impl std::fmt::Debug for Solid {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(f, "Solid({}, faces={}, edges={})", self.id(), self.iter_face().count(), self.iter_edge().count())
	}
}

impl SolidStruct for Solid {
	type Edge = Edge;
	type Face = Face;

	fn id(&self) -> u64 {
		self.shape.id()
	}

	// ==================== Constructors ====================

	fn cube(corner0: DVec3, corner1: DVec3) -> Solid {
		Self::built(apply(Algorithm::Box { corner: corner0, opposite: corner1 }).expect("box: OCCT refused the primitive").shape)
	}

	fn cylinder(r: f64, height: DVec3) -> Solid {
		Self::built(apply(Algorithm::Cylinder { base: DVec3::ZERO, axis: height, radius: r, height: height.length() }).expect("cylinder: OCCT refused the primitive").shape)
	}

	fn sphere(radius: f64) -> Solid {
		Self::built(apply(Algorithm::Sphere { center: DVec3::ZERO, radius }).expect("sphere: OCCT refused the primitive").shape)
	}

	fn cone(r1: f64, r2: f64, height: DVec3) -> Solid {
		Self::built(apply(Algorithm::Cone { base: DVec3::ZERO, axis: height, radius_at_base: r1, radius_at_top: r2, height: height.length() }).expect("cone: OCCT refused the primitive").shape)
	}

	fn torus(r1: f64, r2: f64, axis: DVec3) -> Solid {
		Self::built(apply(Algorithm::Torus { center: DVec3::ZERO, axis, major_radius: r1, minor_radius: r2 }).expect("torus: OCCT refused the primitive").shape)
	}

	fn half_space(plane_origin: DVec3, plane_normal: DVec3) -> Solid {
		Self::built(apply(Algorithm::HalfSpace { origin: plane_origin, normal: plane_normal }).expect("half space: OCCT refused the primitive").shape)
	}

	// ==================== Topology iteration ====================

	fn iter_edge(&self) -> impl Iterator<Item = &Edge> + '_ {
		self.shape.iter_edge()
	}

	fn iter_face(&self) -> impl Iterator<Item = &Face> + '_ {
		self.shape.iter_face()
	}

	fn iter_history(&self) -> impl Iterator<Item = [u64; 2]> + '_ {
		self.history.iter().copied()
	}

	// ==================== Extrude ====================

	fn extrude<'a>(profile: impl IntoIterator<Item = &'a Edge>, dir: DVec3) -> Result<Self, Error> {
		let wire = Self::wire(profile, |_| Error::Extrude)?;
		let face = apply(Algorithm::Face { wire: &wire }).map_err(|_| Error::Extrude)?.shape;
		let prism = apply(Algorithm::Prism { base: &face, vector: dir }).map_err(|_| Error::Extrude)?;
		Self::single(prism, |_| Error::Extrude).map(|(shape, _)| Self::built(shape))
	}

	// ==================== Shell ====================

	fn shell<'a>(&self, thickness: f64, open_faces: impl IntoIterator<Item = &'a Face>) -> Result<Self, Error> {
		const TOLERANCE: f64 = 1e-6;
		let open_faces: Vec<&Face> = open_faces.into_iter().collect();
		let refuse = |message: String| Error::Shell(format!("thickness={thickness} incompatible with the geometry, or self-intersecting offset ({} open face(s)): {message}", open_faces.len()));
		if !open_faces.is_empty() {
			return self.derived(Algorithm::ThickSolid { shape: &self.shape, open_faces: &open_faces, thickness, tolerance: TOLERANCE, join: JoinType::Arc }, refuse);
		}
		// Sealed: the offset shell and the original bound the wall between
		// them; which one is outside follows the sign of the thickness.
		let offset = apply(Algorithm::OffsetShape { shape: &self.shape, offset: thickness, tolerance: TOLERANCE, join: JoinType::Arc, intersection: false }).map_err(|error| refuse(error.to_string()))?.shape;
		let (mut original, mut offset) = (self.shape.components(ShapeKind::Shell), offset.components(ShapeKind::Shell));
		let (Some(original), Some(offset)) = (original.pop(), offset.pop()) else {
			return Err(refuse("no shell to thicken".into()));
		};
		let (outer, inner) = if thickness < 0.0 { (&original, &offset) } else { (&offset, &original) };
		self.derived(Algorithm::Solid { shell: outer, cavities: &[inner] }, refuse)
	}

	// ==================== Fillet / Chamfer ====================

	fn fillet_edges<'a>(&self, radius: f64, edges: impl IntoIterator<Item = &'a Edge>) -> Result<Self, Error> {
		let edges: Vec<&Edge> = edges.into_iter().collect();
		if edges.is_empty() {
			return Ok(self.clone_handle());
		}
		self.derived(Algorithm::Fillet { shape: &self.shape, edges: &edges, radius, law: &[] }, |message| Error::Fillet(format!("radius={radius} does not fit the local geometry on {} edge(s): {message}", edges.len())))
	}

	fn chamfer_edges<'a>(&self, distance: f64, edges: impl IntoIterator<Item = &'a Edge>) -> Result<Self, Error> {
		let edges: Vec<&Edge> = edges.into_iter().collect();
		if edges.is_empty() {
			return Ok(self.clone_handle());
		}
		self.derived(Algorithm::Chamfer { shape: &self.shape, edges: &edges, references: &[], bevel: Bevel::Symmetric { distance } }, |message| Error::Chamfer(format!("distance={distance} does not fit the local geometry on {} edge(s): {message}", edges.len())))
	}

	// ==================== Sweep ====================

	fn sweep<'a, 'b>(profile: impl IntoIterator<Item = &'a Edge>, spine: impl IntoIterator<Item = &'b Edge>, orient: ProfileOrient) -> Result<Self, Error> {
		let refuse = |message: String| Error::Sweep(format!("profile could not be swept along the spine: {message}"));
		let section = Self::wire(profile, refuse)?;
		let spine = Self::wire(spine, refuse)?;
		let frame = match orient {
			ProfileOrient::Fixed => Frame::Fixed,
			ProfileOrient::Torsion => Frame::Frenet,
			ProfileOrient::Up(up) => Frame::Up(up),
		};
		let swept = apply(Algorithm::PipeShell { spine: &spine, sections: &[&section], frame, law: &[], tolerance: None, solid: true }).map_err(|error| refuse(error.to_string()))?;
		Self::single(swept, refuse).map(|(shape, _)| Self::built(shape))
	}

	// ==================== Loft / ThruSections ====================

	fn loft<'a, I: IntoIterator<Item = &'a Edge>, S: IntoIterator<Item = I>>(sections: S, ruled: bool) -> Result<Self, Error>
	where
		Edge: 'a,
	{
		const TOLERANCE: f64 = 1e-7;
		let wires = sections.into_iter().enumerate().map(|(index, section)| Self::wire(section, |_| Error::Loft(format!("loft: section {index} is empty (each section must contain ≥1 edge)")))).collect::<Result<Vec<_>, _>>()?;
		if wires.len() < 2 {
			return Err(Error::Loft(format!("loft: need ≥2 sections, got {} (a single section has no thickness to skin across)", wires.len())));
		}
		let refuse = |message: String| Error::Loft(format!("loft: OCCT BRepOffsetAPI_ThruSections failed (sections={}, ruled={ruled}): {message}. Check that each section forms a valid closed wire and sections are not coplanar.", wires.len()));
		let sections: Vec<&Shape> = wires.iter().collect();
		let lofted = apply(Algorithm::ThruSections { sections: &sections, ruled, tolerance: TOLERANCE, solid: true }).map_err(|error| refuse(error.to_string()))?;
		Self::single(lofted, refuse).map(|(shape, _)| Self::built(shape))
	}

	// ==================== Sew ====================

	fn sew<'a>(faces: impl IntoIterator<Item = &'a Face>, tolerance: f64) -> Result<Self, Error>
	where
		Face: 'a,
	{
		let faces: Vec<&Face> = faces.into_iter().collect();
		if faces.is_empty() {
			return Err(Error::Sew("sew: no faces given (need a face set forming one closed shell)".into()));
		}
		let refuse = |message: String| Error::Sew(format!("sew: {} faces do not form exactly one closed shell within tolerance {tolerance} (gaps, overlaps, multiple shells, or stray faces): {message}", faces.len()));
		let sewn = apply(Algorithm::Sew { faces: &faces, tolerance }).map_err(|error| refuse(error.to_string()))?.shape;
		let mut shells = sewn.components(ShapeKind::Shell);
		let shell = match (shells.pop(), shells.is_empty()) {
			(Some(shell), true) if shell.is_closed() => shell,
			_ => return Err(refuse("the sewn shape is not one closed shell".into())),
		};
		let solid = apply(Algorithm::Solid { shell: &shell, cavities: &[] }).map_err(|error| refuse(error.to_string()))?;
		Self::single(solid, refuse).map(|(shape, _)| Self::built(shape))
	}

	// ==================== Offset surface ====================

	fn offset<'a>(&self, offset: f64, faces: impl IntoIterator<Item = &'a Face>, tolerance: f64) -> Result<Self, Error> {
		let faces: Vec<&Face> = faces.into_iter().collect();
		let refuse = |message: String| Error::Offset(format!("offset: OCCT BRepOffset_MakeOffset failed (offset={offset}, tolerance={tolerance}): {message}. Thin walls/slots whose local thickness is ≤ 2|offset| self-intersect and are rejected."));
		let result = apply(Algorithm::OffsetFaces { shape: &self.shape, faces: &faces, offset, tolerance }).map_err(|error| refuse(error.to_string()))?.shape;
		// A closed shell result is a solid that was not closed up.
		let mut solids = result.components(ShapeKind::Solid);
		let solid = match (solids.pop(), solids.is_empty()) {
			(Some(solid), true) => solid,
			(None, _) => {
				let mut shells = result.components(ShapeKind::Shell);
				match (shells.pop(), shells.is_empty()) {
					(Some(shell), true) if shell.is_closed() => apply(Algorithm::Solid { shell: &shell, cavities: &[] }).map_err(|error| refuse(error.to_string()))?.shape,
					_ => return Err(refuse("the offset is not one closed shell".into())),
				}
			}
			_ => return Err(refuse("the offset holds several solids".into())),
		};
		Ok(Self::built(solid))
	}

	// ==================== Bspline ====================

	fn bspline(u: usize, v: usize, u_periodic: bool, point: impl Fn(usize, usize) -> DVec3) -> Result<Self, Error> {
		if u < 2 || v < 3 {
			return Err(Error::Bspline(format!("grid must be at least 2x3 (u={}, v={})", u, v)));
		}
		let mut coords = Vec::with_capacity(3 * u * v);
		for i in 0..u {
			for j in 0..v {
				coords.extend(point(i, j).to_array());
			}
		}
		let shape = super::ffi::make_bspline_solid(&coords, u as u32, v as u32, u_periodic);
		if shape.is_null() {
			return Err(Error::Bspline(format!("OCCT construction failed (u={}, v={}, u_periodic={})", u, v, u_periodic)));
		}
		Ok(Self::built(Shape::new(shape)))
	}

	// ==================== Clean ====================

	fn clean(&self) -> Result<Self, Error> {
		self.derived(Algorithm::Unify { shape: &self.shape }, |_| Error::Clean)
	}

	// ==================== Boolean primitive ====================

	fn boolean_operand(&self) -> Self {
		self.clone_handle()
	}

	fn boolean_build(b: &Boolean<Self>) -> Result<Vec<Self>, Error> {
		let solids = b.expression.operands();
		if solids.is_empty() {
			return Ok(Vec::new());
		}
		let expression = b.expression.map(Solid::as_shape);
		let applied = apply(Algorithm::Boolean { expression: &expression })?;
		let history = applied.history();
		let shape = applied.shape;

		#[cfg(feature = "color")]
		let colormap: std::collections::HashMap<u64, crate::common::color::Color> = history.iter().filter_map(|[post, source]| solids.iter().find_map(|solid| solid.colormap.get(source)).map(|&color| (*post, color))).collect();
		// No history carries a solid colour -- the result volume descends from
		// no single operand. Take the left operand's, as Fusion 360 does.
		#[cfg(feature = "color")]
		let solid_color = solids[0].colormap.get(&solids[0].id()).copied();

		let history: Arc<[[u64; 2]]> = history.into();
		#[cfg_attr(not(feature = "color"), allow(unused_mut))]
		let mut out: Vec<Solid> = shape
			.components(ShapeKind::Solid)
			.into_iter()
			.map(|solid| {
				Solid::new(
					solid,
					#[cfg(feature = "color")]
					colormap.clone(),
					history.clone(),
				)
			})
			.collect();
		#[cfg(feature = "color")]
		if let Some(color) = solid_color {
			for solid in &mut out {
				let id = solid.id();
				solid.colormap_mut().insert(id, color);
			}
		}
		Ok(out)
	}

	// --- I/O (delegates to super::io helpers) ---

	fn read_step<R: std::io::Read>(reader: &mut R) -> Result<Vec<Self>, Error> {
		super::io::read_step(reader)
	}

	fn read_brep<R: std::io::Read>(reader: &mut R) -> Result<Vec<Self>, Error> {
		super::io::read_brep(reader)
	}

	fn write_step<'a, W: std::io::Write>(solids: impl IntoIterator<Item = &'a Self>, writer: &mut W) -> Result<(), Error>
	where
		Self: 'a,
	{
		super::io::write_step(solids, writer)
	}

	fn write_brep<'a, W: std::io::Write>(solids: impl IntoIterator<Item = &'a Self>, writer: &mut W) -> Result<(), Error>
	where
		Self: 'a,
	{
		super::io::write_brep(solids, writer)
	}

	fn mesh<'a>(solids: impl IntoIterator<Item = &'a Self>, options: crate::traits::Tessellation) -> Result<crate::common::mesh::Mesh, Error>
	where
		Self: 'a,
	{
		super::io::mesh(solids, options)
	}

	// ==================== Queries ====================

	fn volume(&self) -> Result<f64, Error> {
		self.shape.volume()
	}

	fn area(&self) -> Result<f64, Error> {
		self.shape.area()
	}

	fn center(&self) -> Result<DVec3, Error> {
		self.shape.center()
	}

	fn inertia(&self) -> Result<glam::DMat3, Error> {
		self.shape.inertia()
	}

	fn contains(&self, point: DVec3) -> bool {
		self.shape.contains(point)
	}

	fn bounding_box(&self) -> [DVec3; 2] {
		self.shape.bounding_box()
	}

	// ==================== Color ====================

	#[cfg(feature = "color")]
	fn color(self, color: impl Into<crate::common::color::Color>) -> Self {
		// Existing face colours are dropped: painting the whole solid is a
		// statement about the whole solid.
		let colormap = std::collections::HashMap::from([(self.shape.id(), color.into())]);
		Self::new(self.shape, colormap, self.history)
	}

	#[cfg(feature = "color")]
	fn color_clear(self) -> Self {
		Self::new(self.shape, std::collections::HashMap::new(), self.history)
	}
}

impl Solid {
	/// The same TShape under a new handle: ids and history kept.
	fn clone_handle(&self) -> Self {
		Self::new(
			self.shape.clone(),
			#[cfg(feature = "color")]
			self.colormap.clone(),
			self.history.clone(),
		)
	}

	/// A rebuilt solid, colours carried by face order and history dropped:
	/// what a transform that rebuilds topology leaves.
	fn rebuilt(&self, shape: Shape) -> Self {
		#[cfg(feature = "color")]
		let colormap = self.remap_colormap_by_order(&shape);
		Self::new(
			shape,
			#[cfg(feature = "color")]
			colormap,
			Default::default(),
		)
	}
}

impl Transform for Solid {
	fn translate(self, translation: DVec3) -> Self {
		// A placement: the TShape is shared and the location changes, so the
		// history's ids stay valid and only the caches have to be renewed.
		Self::new(
			self.shape.translated(translation),
			#[cfg(feature = "color")]
			self.colormap,
			self.history,
		)
	}

	fn rotate(self, axis_origin: DVec3, axis_direction: DVec3, angle: f64) -> Self {
		Self::new(
			self.shape.rotated(axis_origin, axis_direction, angle),
			#[cfg(feature = "color")]
			self.colormap,
			self.history,
		)
	}

	// scale/mirror cannot be placements: since OCCT fix 0027457 (v7.6) a
	// location rejects a scale != 1 or a negative determinant, because the
	// downstream algorithms break on non-rigid locations. They rebuild.

	fn scale(self, center: DVec3, factor: f64) -> Self {
		let offset = center * (1.0 - factor);
		let matrix = [factor, 0.0, 0.0, offset.x, 0.0, factor, 0.0, offset.y, 0.0, 0.0, factor, offset.z];
		let shape = apply(Algorithm::Transform { shape: &self.shape, matrix }).expect("scale: OCCT refused the transform").shape;
		self.rebuilt(shape)
	}

	fn mirror(self, plane_origin: DVec3, plane_normal: DVec3) -> Self {
		let n = plane_normal.normalize();
		let reflect = glam::DMat3::IDENTITY - 2.0 * glam::DMat3::from_cols(n * n.x, n * n.y, n * n.z);
		let offset = plane_origin - reflect * plane_origin;
		let row = |index: usize| [reflect.row(index).x, reflect.row(index).y, reflect.row(index).z, offset[index]];
		let matrix = [row(0), row(1), row(2)].concat().try_into().expect("twelve coefficients");
		let shape = apply(Algorithm::Transform { shape: &self.shape, matrix }).expect("mirror: OCCT refused the transform").shape;
		self.rebuilt(shape)
	}
}

impl Clone for Solid {
	fn clone(&self) -> Self {
		// A deep copy rebuilds topology: history names faces that no longer
		// exist, so it is dropped rather than remapped.
		self.rebuilt(self.shape.deep_copy())
	}
}
