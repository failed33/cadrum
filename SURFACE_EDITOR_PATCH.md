# Surface Editor binding patch

Upstream: https://github.com/lzpel/cadrum, crates.io release 0.8.18.
Upstream source and MIT license are retained. OCCT has its own license.

This copy allows adding missing bindings without changing Cargo registry files.
Only `cad-kernel` may depend on it. Do not add project or vessel policy here.

Local additions bind OCCT BRep validity analysis and a pipe sweep driven by a
scalar law. Absolute tessellation uses OCCT's surface-deflection controls and
`BRepLib::UpdateDeflection`. Refinement responds to measured chord error rather
than a fixed pass count. It stops on convergence, an unchanged mesh across three
passes, the kernel's resolution, or four million triangles before further
refinement; errors retain requested/achieved deflection and native mesh status.
The triangle limit bounds further refinement, not OCCT's allocation within a pass.
Meshing exceptions are translated by CXX. These invoke OCCT algorithms and do
not implement geometry locally. The deflection measurement samples the
triangulation; it is not a certified global Hausdorff bound.
Absolute tessellation also uses `Poly_MergeNodesTool` on each temporary face
mesh to consolidate singular nodes and remove collapsed facets (notably sphere
poles). Its merge distance is bounded by OCCT's confusion tolerance and 0.1%
of the requested chord tolerance; that distance is reserved from the meshing
deflection budget. This changes only authored CAD display/bake triangulation,
not analytic geometry or imported anatomical meshes. Face ancestry remains
attached to every emitted triangle.
Spline interpolation accepts an explicit tolerance. Native BRep writing excludes
display triangulation so preview rebuilding does not mutate archived geometry.
Track local changes against the registry source when updating this dependency.
Remove this copy when a compatible upstream release supplies the needed APIs.

The sweep binding also returns the start/end boundary identities supplied by
`BRepOffsetAPI_MakePipeShell::FirstShape` and `LastShape`. These remain live
native identities inside `cad-kernel`. Application construction references use
generating-feature roles, never these identities or enumeration positions.

Application integration also adds exception-safe box, sphere and cylinder
constructors and an affine geometry transform through OCCT's
`BRepBuilderAPI_GTransform`. The latter rebuilds topology for nonuniform scale;
old native subshape references are not carried across it. These operations expose
existing OCCT algorithms. Their binding values remain private to `cad-kernel`.

Affine transforms now return OCCT `BRepBuilderAPI_GTransform::Modified` edge
history alongside the transformed solid. `cad-kernel` translates that history
into new evaluation-scoped generated boundary references. Old live references
remain stale; neither native edge identities nor enumeration positions become
persisted selection.

## Evaluated graph correspondence continuation

The shared builder history relay now records faces and edges, retaining multiple
source relations when OCCT merges topology. The deep-copy relay uses
`BRepBuilderAPI_Copy::ModifiedShape`; traversal order is no longer treated as
copy correspondence. This supports surviving-edge resolution across native trim
and fillet operations. The application keeps this history inside its worker-local
evaluated graph. It does not serialize native identities as durable selection.

## Open surfaces

A shell carrier now sits beside `Solid`. `Shell` is a single `TopAbs_SHELL`,
open or closed, that is never upgraded to a solid, so `Solid`'s closed,
positive-volume guarantee is unchanged and neither type can produce the other.
It binds `BRepBuilderAPI_Sewing` without the `BRepBuilderAPI_MakeSolid` step,
`BRepOffsetAPI_MakeOffsetShape` in skin mode, `BRepOffsetAPI_MakeFilling` for a
boundary loop, and `ShapeAnalysis_FreeBounds` for the loops that remain open.
Every tolerance, continuity, degree and join type is a parameter of the call;
the binding holds no defaults of its own, and the failure of any of these
entry points is an OCCT message carried out through the crate's `Error`, never
a null shape. No new OCCT toolkit is linked and no geometry is computed here.

The carrier these wrap is kind-generic: it holds any `TopoDS_Shape` and reports
its kind rather than asserting one, and BRep reading, compound assembly,
decomposition and triangulation take the kind as an argument. A BRep payload
can therefore be read as shells without the `TopAbs_SOLID` filter that
`Solid::read_brep` keeps. `cad-kernel` tags the archive with the kind it wrote.

## Native failures

CXX translates ordinary `Standard_Failure` and C++ exceptions into Rust errors.
The bridge installs no process-wide signal handlers and performs no long jumps
past native destructors. A collinear filling boundary is identified through
OCCT's principal line moments before its plate builder dereferences the absent
initial surface. Collapsed offset results are refused as empty geometry.

This is an in-process kernel, not isolation against arbitrary native memory
corruption. A future crash-containment requirement needs a restartable process;
a returned segmentation-fault error cannot make partially mutated native state safe.

## Sweep geometry and numerical properties

Auxiliary guides require `GuideCorrespondence::NormalPlane` or `ArcLength`.
Normal-plane correspondence keeps sections normal to the spine; arc-length
correspondence may tilt otherwise rigid sections. The latter must not be tested
against area times spine length without accounting for that projection.
Linear/boundary tolerance and angular tolerance remain separate; a requested
linear tolerance no longer overwrites the angular tolerance.

`patches/GeomFill_GuideTrihedronAC.cxx` is OCCT V8_0_1's implementation with two
second-derivative corrections: normalization uses the squared first derivative,
and the guide-parameter chain rule uses the squared station scaling. It is
compiled into the bridge object, providing the complete class implementation
before the static OCCT archive is needed. Both prebuilt and source builds get
the same correction; no cached archive is edited. The version assertion requires
reassessment when OCCT changes. The original LGPL header is retained; this
upstream-derived file is not covered by cadrum's MIT license.

Source: https://github.com/Open-Cascade-SAS/OCCT/blob/V8_0_1/src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GuideTrihedronAC.cxx

The sweep builder checks its achieved approximation error and increases its
segment budget if necessary, up to 1600 segments. The derivative correction
allows the periodic-guide regression to meet its original construction accuracy
at the default segment budget.

Volume, area, centre and inertia queries now return `Result`. Adaptive Gauss
integration is attempted first; volume properties fall back to span-aware
Gauss–Kronrod if needed. The bridge checks the returned error estimate against
1e-6 instead of discarding it. This is an integration estimate, not a certificate
of shape accuracy or independent error bounds for every moment. Analytic and
transformation tests cover the moments. The application caches accepted area
and volume on its immutable shape, so public property reads remain infallible.

Tests retain the 60-point NACA sections and use the independent analytic thickness
integral. Normal-plane sweeps are checked against Pappus; an oblique guide has
an analytic projected-area reference for arc-length mode. Absolute tessellation
of the closed ring is exercised below the former 0.03 cutoff.

## Boolean expression ownership

`boolean-expression/` is a dependency-free Rust crate shared with the application's
mesh evaluator. It owns compact postfix expressions and the three set operations;
operand geometry, preparation and provenance belong to each backend. Construction
of the program guarantees stack arity and operand indices; those are assertions,
not a second input-validation pipeline.

`Algorithm::Boolean` and the public solid operators now use one CellsBuilder path.
The binding evaluates the expression against OCCT's split-cell membership index,
selects those cells, removes internal boundaries, and rebuilds OCCT history before
copying the result. No Cartesian DNF expansion or geometric classifier is involved.
The protected CellsBuilder membership/material fields are specific to the pinned
OCCT 8.0.1 implementation and need review when that dependency changes.

The former raw DIMACS `Solid::boolean` entry and two-group algorithm row are removed.
Use `Boolean` operators or `Expression` with `Algorithm::Boolean`. Empty expressions
build an empty vector; `build()` still requires one solid. Errors retain their native
cause, and result pieces share one immutable history allocation.
