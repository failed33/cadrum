# Upstream OCCT limitations

These standalone reproducers isolate dependency defects. They do not implement
application behavior or belong to the application regression gate.

## Spline extrusion integral

`spline-integral.cpp` constructs a periodic 16-point elliptical spline and
extrudes its planar face by 3. The independent reference samples the same spline
at 65,536 parameters and applies the planar polygon area formula.

On the published OCCT 8.0.1 rev2 Linux binary, adaptive native volume is
17.349229117 versus 18.853715396 from the sampled boundary, despite a small
reported convergence error. The executable returns 1 while this defect persists.
The application spline extrusion test instead verifies the emitted geometry
against the sampled profile, without changing the native integrator.

To reproduce with an extracted published package (set `occt_install` to its root):

```sh
c++ -std=c++17 -O2 -I"$occt_install/include/opencascade" \
  vendor/cadrum/diagnostics/spline-integral.cpp \
  -Wl,--start-group "$occt_install"/lib/libTK*.a -Wl,--end-group \
  -lpthread -ldl -o /tmp/spline-integral
/tmp/spline-integral
```

This link command is for Linux. No replacement OCCT symbols are compiled.

## Removed auxiliary-guide behavior

The removed guide extension required correcting OCCT's arc-length guide
trihedron second derivatives (normalization's first-derivative norm and the
squared guide-parameter station scale). It was unused by application commands.
Its API and feature tests were removed together; no other frame is silently
substituted. The fork's pre-removal `tests/solid_sweep.rs` and derivative changes
are recorded in git history (`2bfeac3`). This is a capability removal, not a claim
that the upstream guide algorithm is fixed.
