# Ported from Bertolazzi Clothoids

`clothoid_g2.cpp` is an extract of Enrico Bertolazzi's **Clothoids** (tag `2.1.0`,
<https://github.com/ebertolazzi/Clothoids>, BSD 2-clause -- see
`Clothoids-license.txt`), cut down to the one operation the planner uses: the
three-arc `G2` Hermite solve.

`AMENT_IGNORE` here keeps the linters off foreign code, whose formatting is
upstream's and must stay that way to remain diffable.

## Why a port rather than a dependency

The full library needs four source trees -- Clothoids, `UtilsLite` (which bundles
its own Eigen and fmt), `GenericContainer`, and `quarticRootsFlocke` -- totalling
~9.8 MB which would have taken super long to build.

## What was taken

- `Fresnel.cc`: the coefficient tables, `FresnelCS`, `evalXYaLarge`,
  `LommelReduced`, `evalXYazero`, `evalXYaSmall`, `GeneralizedFresnelCS`,
  `ClothoidData::build_G1`, `ClothoidData::origin_at`.
- `ClothoidG2.cc`: `G2solve3arc::build`, `evalFJ`, `evalF`, `build_solution`,
  `solve`.
- `G2lib.cc`: `rangeSymm`, `Solve2x2`, `m_1_sqrt_pi`.

The arithmetic is copied, not rewritten. Changes are structural: `ClothoidCurve`/
`ClothoidData` collapsed into one `CData`, `G2solve3arc` renamed `Solver`,
`UTILS_ASSERT` and the fmt-formatted error machinery (what dragged in
`UtilsLite`) removed, and `solve()`'s `catch(...)` no longer writes to `std::cerr`
-- failure already comes back through the return value.
