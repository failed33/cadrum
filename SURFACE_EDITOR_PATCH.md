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
