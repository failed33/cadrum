# Surface Editor binding patch

Upstream: https://github.com/lzpel/cadrum, crates.io release 0.8.18.
Upstream source and MIT license are retained. OCCT has its own license.

This copy allows adding missing bindings without changing Cargo registry files.
Only `cad-kernel` may depend on it. Do not add project or vessel policy here.

Local additions bind OCCT BRep validity analysis and a pipe sweep driven by a
scalar law. Absolute tessellation uses OCCT's surface-deflection controls and
`BRepLib::UpdateDeflection`, with at most six refinements before refusal.
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
