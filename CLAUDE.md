# CLAUDE.md — lib/math

## Purpose

`math` is Layer 2 in the Ka-Boost dependency stack. It fills the gaps between FANUC Karel's sparse built-in math library and what robot kinematics, geometry, and path-planning algorithms actually need. Core additions: exponentiation, logarithms, floor/ceil/round, arc/angle conversions, hyperbolic functions, a seeded linear-congruential PRNG, array statistics and quicksort, vector algebra (norm, projection, Manhattan distance), intrinsic pose rotations/translations, and two GPP-based generic sorting classes (`arraysort.klc`, `pathsort.klc`).

Used by: `matrix`, `hash`, `queue`, `iterator`, `graph`, `files`, `csv`, `shapes`, `pose`, `sensors`, `draw`, `paths` — i.e., virtually everything above Layer 1.

---

## Repository Layout

```
lib/math/
├── package.json                  # rossum manifest: name, version, deps, tp-interfaces
├── include/
│   ├── math.klt                  # Public constants (M_PI, M_E, EPSILON, MAX_INT, etc.)
│   ├── math.klh                  # Public function declarations (47 routines)
│   ├── math.builtins.klh         # Declarations for built-in wrappers (9 routines)
│   ├── math.private.klh          # Private declarations (quicksort internals)
│   ├── math.private.klt          # PRNG parameters + m_seed global var
│   └── classes/
│       ├── arraysort.klc         # GPP generic quicksort/bubblesort/dedup for ARRAY[*]
│       ├── arraysort.klh         # Declarations for arraysort class
│       ├── pathsort.klc          # Same algorithms for Karel PATH (linked list)
│       └── pathsort.klh          # Declarations for pathsort class
├── src/
│   └── math.kl                   # All implementation — single compilation unit
└── test/
    ├── test_math.kl              # 18 tests, 43 assertions (KUnit)
    ├── test_pth_srt.kl           # 6 tests for arraysort/pathsort classes
    ├── test_ls_math.ls           # TP cross-over test for tp-interface routines
    └── test_types/
        ├── testarr.klt           # Config for arraysort with t_TST_ARR
        └── testpath.klt          # Config for pathsort with STD_PTH_NODE
```

---

## Full API Reference

### Constants — `math.klt`

| Constant | Value | Notes |
|---|---|---|
| `M_E` | 2.71828182845... | Euler's number |
| `M_LOG2E` | 1.44269504... | log₂ e |
| `M_LOG10E` | 0.43429448... | log₁₀ e |
| `M_LN2` | 0.69314718... | logₑ 2 |
| `M_LN10` | 2.30258509... | logₑ 10 |
| `M_PI` | 3.14159265... | π |
| `M_2PI` | 6.28318530... | 2π |
| `M_PI_2` | 1.57079632... | π/2 |
| `M_PI_4` | 0.78539816... | π/4 |
| `M_1_PI` | 0.31830988... | 1/π |
| `M_2_PI` | 0.63661977... | 2/π |
| `M_2_SQRTPI` | 1.12837916... | 2/√π |
| `M_SQRT2` | 1.41421356... | √2 |
| `M_SQRT1_2` | 0.70710678... | 1/√2 |
| `M_RAD2DEG` | 57.29577951 | multiply radians → degrees |
| `M_DEG2RAD` | 0.0174532925 | multiply degrees → radians |
| `MAX_RAND` | 13849 | PRNG modulus remainder |
| `MAX_INT` | 32767 | Karel signed short max |
| `MAX_BYTE` | 255 | byte max |
| `EPSILON` | 0.0001 | near-zero threshold (defined in errors.klt, used throughout) |

### Built-in Wrappers — `math.builtins.klh` / `src/math.kl`

All wrap FANUC Karel built-ins. Some add epsilon-guard to prevent SQRT(0) or ATAN2(0,0).

| Function | Signature | Notes |
|---|---|---|
| `math__abs` | `(val:REAL):REAL` | ABS() wrapper |
| `math__exp` | `(val:REAL):REAL` | EXP() wrapper |
| `math__sqrt` | `(val:REAL):REAL` | Returns 0 if \|val\| < EPSILON |
| `math__sin` | `(angle:REAL):REAL` | SIN() — angle in degrees |
| `math__cos` | `(angle:REAL):REAL` | COS() — angle in degrees |
| `math__tan` | `(angle:REAL):REAL` | TAN() — angle in degrees |
| `math__ln` | `(val:REAL):REAL` | LN() wrapper |
| `math__atan2` | `(opposite:REAL; adjacent:REAL):REAL` | Returns 0 if both < EPSILON; note: ATAN2(adj, opp) internally |
| `math__atan_pos` | `(pose:XYZWPR; grp_no:INTEGER):REAL` | Returns ATAN2(pose.x, pose.y); guards zero |

### Logarithmic / Exponent

| Function | Signature | Formula | Notes |
|---|---|---|---|
| `math__pow` | `(value, exponent:REAL):REAL` | EXP(exp * LN(val)) | Fails for value ≤ 0 (LN domain) |
| `math__log10` | `(num:REAL):REAL` | LN(num)/M_LN10 | |
| `math__log2` | `(num:REAL):REAL` | LN(num)/M_LN2 | |

### Rounding / Formatting

| Function | Signature | Returns | Notes |
|---|---|---|---|
| `math__floor` | `(num:REAL):INTEGER` | Largest int ≤ num | Handles negatives via TRUNC-1 |
| `math__ceil` | `(num:REAL):INTEGER` | Smallest int ≥ num | Handles positives via TRUNC+1 |
| `math__round` | `(num:REAL):INTEGER` | Nearest int | Uses ≥0.5 threshold |
| `math__decimal` | `(num:REAL; digits:INTEGER):REAL` | REAL rounded to N places | Uses CNV_REAL_STR; fixes leading-zero and minus-dot edge cases |

### Arc / Angle

| Function | Signature | Formula |
|---|---|---|
| `math__arclength` | `(degrees, radius:REAL):REAL` | radius × degrees × π/180 |
| `math__arcangle` | `(length, radius:REAL):REAL` | length/radius × 180/π |
| `math__map_to_360` | `(angle:REAL):REAL` | Clamps to [0,360]: add/sub 360 once |

### Hyperbolic

| Function | Signature | Formula |
|---|---|---|
| `math__cosh` | `(x:REAL):REAL` | 0.5×(EXP(x)+EXP(−x)) |
| `math__sinh` | `(x:REAL):REAL` | 0.5×(EXP(x)−EXP(−x)) |

### Miscellaneous

| Function | Signature | Notes |
|---|---|---|
| `math__bitmask` | `(num:INTEGER):INTEGER` | Returns 2^(num−1) via EXP/LN |

### Linear Mapping

| Function | Signature | Formula |
|---|---|---|
| `math__map_real` | `(x, in_min, in_max, out_min, out_max:REAL):REAL` | (x−in_min)×(out_max−out_min)/(in_max−in_min)+out_min |
| `math__map_int` | `(x, in_min, in_max:REAL; out_min, out_max:INTEGER):INTEGER` | TRUNC of map_real result |

### PRNG — Linear Congruential Generator

Parameters (from `math.private.klt`): M=65536, A=25173, C=13849. State stored in global `m_seed:INTEGER`.

| Function | Signature | Returns | Notes |
|---|---|---|---|
| `math__srand` | `(seed:INTEGER)` | — | Sets m_seed = seed MOD 65535 |
| `math__rand` | `():REAL` | [0.0, 1.0) | Updates m_seed each call |
| `math__rand_range` | `(min, max:REAL):REAL` | [min, max) | map_real(rand,0,1,min,max) |
| `math__rand_int` | `():INTEGER` | [0, MAX_INT) | map_int(rand,0,1,0,MAX_INT) |
| `math__rand_color` | `():INTEGER` | [0, 255) | map_int(rand,0,1,0,MAX_BYTE) |
| `math__rand_position` | `(seed:INTEGER):XYZWPR` | Random pose | 300×300×300 mm box, WPR [0°,90°]; calls srand(seed×i) per axis |
| `math__rand_vector` | `(seed:INTEGER):VECTOR` | Random vector | 300×300×300 mm box; srand(seed×i) per dimension |
| `math__rand_rarr` | `(seed:INTEGER; out_arr:ARRAY[*] OF REAL)` | — | Fills array; srand(seed×i) per element |

**Critical:** PRNG state is global. Calling `math__srand` from one routine affects all subsequent `math__rand` calls in the program. `rand_position` / `rand_vector` / `rand_rarr` re-seed internally on each element, so calling them re-rolls the global state.

### Array Statistics

All array functions use `ARRAY_LEN()` for bounds. `sum_real` and `average_real` skip `UNINIT()` elements.

| Function | Signature | Returns |
|---|---|---|
| `math__max_int` | `(numbers:ARRAY[*] OF INTEGER):INTEGER` | Maximum value |
| `math__max_int_index` | `(numbers:ARRAY[*] OF INTEGER):INTEGER` | Index of maximum |
| `math__min_int` | `(numbers:ARRAY[*] OF INTEGER):INTEGER` | Minimum value |
| `math__min_int_index` | `(numbers:ARRAY[*] OF INTEGER):INTEGER` | Index of minimum |
| `math__max_real` | `(numbers:ARRAY[*] OF REAL):REAL` | Maximum value |
| `math__max_real_index` | `(numbers:ARRAY[*] OF REAL):INTEGER` | Index of maximum |
| `math__min_real` | `(numbers:ARRAY[*] OF REAL):REAL` | Minimum value |
| `math__min_real_index` | `(numbers:ARRAY[*] OF REAL):INTEGER` | Index of minimum |
| `math__sum_real` | `(numbers:ARRAY[*] OF REAL):REAL` | Sum (skips UNINIT) |
| `math__average_real` | `(numbers:ARRAY[*] OF REAL):REAL` | Mean (skips UNINIT, returns 0 if n=0) |

### Sorting (Built-in)

| Function | Signature | Notes |
|---|---|---|
| `math__quicksort_real` | `(arr:ARRAY[*] OF REAL)` | In-place ascending sort; calls quicksort_r(arr,1,ARRAY_LEN) |
| `math__quicksort_int` | `(arr:ARRAY[*] OF INTEGER)` | In-place ascending sort; calls quicksort_i(arr,1,ARRAY_LEN) |

### Vector Math

| Function | Signature | Returns | Notes |
|---|---|---|---|
| `math__norm` | `(v:VECTOR):REAL` | √(x²+y²+z²) | Full Euclidean norm |
| `math__norm2` | `(v:VECTOR):REAL` | x²+y²+z² | Squared norm — avoids SQRT for comparison |
| `math__proj` | `(v, ax:VECTOR):VECTOR` | Projection of v onto ax | (v@ax)/norm2(ax) × ax |
| `math__proj_orthoganal` | `(v, ax:VECTOR):VECTOR` | Component of v perp. to ax | v − proj(v,ax) |
| `math__proj_length` | `(v, ax:VECTOR):REAL` | Scalar projection length | \|v@ax\|/norm(ax) |
| `math__average_vector` | `(arrv:ARRAY[*] OF VECTOR):VECTOR` | Component-wise mean | Skips UNINIT elements |
| `math__manhattan_dist` | `(start_v, end_v:VECTOR):REAL` | L1 distance | \|Δx\|+\|Δy\|+\|Δz\| |

### Pose Transformations

All use the FANUC `:` operator (frame/pose multiplication — homogeneous transform composition).

**Translation:**

| Function | Signature | Effect |
|---|---|---|
| `math__translate` | `(pos:XYZWPR; amount:VECTOR):XYZWPR` | pos : {amount.x, amount.y, amount.z, 0,0,0} — translate in local frame |

**Intrinsic Rotations (local-frame, around current tool axes):**

| Function | Signature | Effect |
|---|---|---|
| `math__rotx` | `(pos:XYZWPR; angle:REAL):XYZWPR` | pos : {0,0,0, angle,0,0} |
| `math__roty` | `(pos:XYZWPR; angle:REAL):XYZWPR` | pos : {0,0,0, 0,angle,0} |
| `math__rotz` | `(pos:XYZWPR; angle:REAL):XYZWPR` | pos : {0,0,0, 0,0,angle} |

**Vector Rotations (applies rotation, discards WPR of result):**

| Function | Signature | Effect |
|---|---|---|
| `math__rotx_vec` | `(pos:VECTOR; angle:REAL):VECTOR` | {0,0,0,angle,0,0} : pos |
| `math__roty_vec` | `(pos:VECTOR; angle:REAL):VECTOR` | {0,0,0,0,angle,0} : pos |
| `math__rotz_vec` | `(pos:VECTOR; angle:REAL):VECTOR` | {0,0,0,0,0,angle} : pos |

---

### Generic Sort Classes

#### `arraysort.klc` — ARRAY Sort Template

Instantiate via GPP. Required `.klt` config must define:

| Parameter | Type | Purpose |
|---|---|---|
| `ARRAYTYPE` | type name | Element type stored in ARRAY |
| `ARRAYEPSILON` | REAL or INTEGER | Equality tolerance |
| `callback(elem:ARRAYTYPE):REAL` | routine | Extract sort key from element |
| `glt(t1,t2,cmp):BOOLEAN` | routine | Greater/less-than comparator |
| `eq(t1,t2):BOOLEAN` | routine | Equality comparator |

**Methods (after `%class myname('arraysort.klc','arraysort.klh','myconfig.klt')`):**

| Method | Signature | Notes |
|---|---|---|
| `myname__partition` | `(list:ARRAY[*] OF ARRAYTYPE; low,high:INTEGER; comparator:BOOLEAN):INTEGER` | Pivot = list[high]; comparator FALSE=asc, TRUE=desc |
| `myname__quicksort` | `(list:ARRAY[*] OF ARRAYTYPE; low,high:INTEGER; comparator:BOOLEAN)` | Recursive |
| `myname__bubblesort` | `(list:ARRAY[*] OF ARRAYTYPE; low,high:INTEGER; comparator:BOOLEAN)` | O(n²) alternative |
| `myname__remove_duplicates` | `(list:ARRAY[*] OF ARRAYTYPE; low,high:INTEGER):INTEGER` | Removes consecutive equal elements; returns new length |
| `myname__keep_unique` | `(list:ARRAY[*] OF ARRAYTYPE; low,high:INTEGER):INTEGER` | Full deduplication; returns new length |

#### `pathsort.klc` — Karel PATH Sort Template

Same algorithm set, but operates on a Karel `PATH` (linked list) instead of `ARRAY[*]`. Required `.klt` config must additionally define `LISTHEADER` (PATH header type) and `LISTTYPE` (node type). Uses `DELETE_NODE` for cleanup rather than array assignment. All methods take `list:PATH` instead of `ARRAY[*]`.

---

## Core Patterns

### Pattern 1: Vector Normalization with Null-Guard

Used throughout `shapes`, `pose`, `draw` to create unit vectors before frame math.

```karel
-- lib/shapes/src/shapes.kl:24
out_plane.normal = normal_vec / math__norm(normal_vec)

-- lib/pose/lib/poselib/pose.kl:914
RETURN(v / math__norm(v))

-- lib/draw/lib/drawlib/draw.kl:132-139
length = math__norm((l1.r1 - l1.r0))
IF length = 0 THEN
  RETURN(FALSE)
ENDIF
n_l1 = (l1.r1 - l1.r0) / length
```

Always check `math__norm(v) = 0` (or `< EPSILON`) before dividing. `math__norm2` avoids SQRT for pure comparisons.

---

### Pattern 2: Chained Rotation + Translation for Coordinate Conversion

Core of the UV→XYZWPR pipeline in `pose`. The `:` operator composes frames; `math__rotx/y/z` and `math__translate` build up the full transform chain.

```karel
-- lib/pose/lib/poselib/pose.kl:935-944
-- Cylindrical (theta, z, r) → Cartesian, rotating around Z axis
SELECT z_axis OF
  CASE(Z_AXES):
    v.x = 0 ; v.y = cyl_pose.y ; v.z = cyl_pose.z
    o = math__translate(math__rotz(origin, cyl_pose.x), v)
  CASE(X_AXES):
    v.x = cyl_pose.y ; v.y = 0 ; v.z = cyl_pose.z
    o = math__translate(math__rotx(origin, cyl_pose.x), v)
ENDSELECT

-- Nested rotations for polar coordinates (two-axis rotation)
o = math__translate(math__roty(math__rotz(origin, pol_pose.x), pol_pose.y), v)
```

Order matters: rotations are applied right-to-left (inner first). `math__translate` appends a translation in the rotated local frame.

---

### Pattern 3: PRNG for Reproducible Test Data

```karel
-- lib/matrix/src/matrix.kl:202-203
GET_TIME(tme)
math__srand(tme * i)
arr[i] = math__rand

-- lib/csv/test/write_test/tst_csv_pipe.kl:38
tme = GET_USEC_TIM
pose = math__rand_position(tme)
tstcsv__write_row_xyzwpr(pose)
```

Seed with timestamp so each test run differs. `math__rand_position` / `math__rand_vector` re-seed per axis; after calling them the global `m_seed` state is at an unpredictable value — call `math__srand` again if you need a specific sequence afterward.

---

### Pattern 4: Generic Sorting with `arraysort.klc`

```karel
-- lib/draw/lib/drawlib/draw.kl:42
-- Instantiate a sorter for t_VERT_CLIP (polygon-line intersection vertices)
%class srtclp('arraysort.klc','arraysort.klh','sortintrsct.klt')

-- Later usage:
srtclp__quicksort(clips, 1, PATH_LEN(clips), FALSE)   -- ascending by callback value
srtclp__keep_unique(clips, 1, PATH_LEN(clips))         -- remove duplicates
```

The `.klt` config must define `ARRAYTYPE`, `ARRAYEPSILON`, and the `callback`, `glt`, `eq` routines. One `%class` expansion produces all five methods.

---

### Pattern 5: Path Tangent + Perpendicular via `math__rotz_vec`

Used in `draw` and `pathlib` to compute raster directions, inset offsets, and nearest-neighbor search vectors.

```karel
-- lib/paths/pathlib/lib/pathlib.kl:66
-- Get the direction perpendicular to raster
parallel_dir = math__rotz_vec(raster_dir, -90)

-- lib/draw/lib/drawlib/draw.kl:100
-- Generate perpendicular vector for polygon inset
v_out = math__rotz_vec(v, 90)

-- lib/draw/lib/drawlib/draw.kl:1424
-- Hexagon vertex generation: rotate to angle, translate, rotate back
p = math__rotz(math__translate(math__rotz(origin, ang), trans), -ang)
```

For 2D problems (z=0 vectors), `math__rotz_vec` is the go-to rotation primitive.

---

### Pattern 6: K-D Tree Distance via Norm (Template Context)

```karel
-- lib/graph/include/class/kd_tree/default_kd_dist.klt:51
ROUTINE distance(point:KDTREE_DATA_TYPE; nde:KDTREE_DATA_TYPE):KDTREE_COMPARE_TYPE
BEGIN
  RETURN(math__norm(nde.KDTREE_NODE_NAME - point.KDTREE_NODE_NAME))
END distance
```

The K-D tree template accepts any distance function. The default wires `math__norm` directly into the spatial search. This pattern of embedding math routines in GPP template callbacks appears throughout graph, pathsort, and arraysort.

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---|---|---|
| Calling `math__pow(0, exp)` | Runtime error / undefined — LN(0) is undefined | Guard: `IF value > EPSILON THEN ... ENDIF` |
| Using `math__rotz` expecting world-frame rotation | Pose drifts after multiple rotations — `:` is local/intrinsic | Use `matpose__pose_to_mat` + matrix multiply for world-frame composition |
| Dividing vector by `math__norm` without zero-check | Division-by-zero runtime error on null vectors | Always check `math__norm(v) > EPSILON` first |
| Calling `math__rand` before `math__srand` | m_seed is 0 → PRNG generates deterministic sequence starting at 0 | Always call `math__srand(seed)` (use GET_TIME) before first `math__rand` |
| Expecting `math__map_to_360` to handle angles outside ±360 | Returns wrong value for e.g. −720° | Only adds/subtracts 360 once; wrap in a loop for large angles |
| Instantiating `arraysort.klc` without defining `callback`, `glt`, `eq` | GPP compile error | All three routines must be defined in the `.klt` config file |
| Using `math__floor`/`math__ceil` on values where TRUNC already gives the right answer | Subtle off-by-one for exactly-integer inputs | Both functions always adjust TRUNC output; works correctly for exact integers too |

---

## Dependencies

**This module depends on:**
- `ktransw-macros` — `declare_function`, `header_guard`, `type_guard` macros
- `Strings` — `CNV_REAL_STR`, `CNV_STR_REAL` (used in `math__decimal`)
- `errors` — `karelError`, `CHK_STAT`, `EPSILON` constant (from errors.klt)
- `systemlib` — `VEC()` macro, `ZEROPOS`, system type macros

**Modules that depend directly on math:**
- `matrix` — random array/matrix init
- `hash` — hash function arithmetic
- `queue` — comparators
- `iterator` — element comparisons
- `graph` — K-D tree distance, TSP cost
- `shapes` — plane/line/collision geometry
- `pose` — IK/FK, cylindrical/polar conversion, quaternions
- `draw` — rasterization, polygon ops
- `paths/*` — pathlib, pathplan, pathmake, pathmotion, pathlayer

---

## Build / Integration Notes

- **Single compilation unit:** All implementation lives in `src/math.kl`. No split `.klc` files for the core routines.
- **`EPSILON` source:** Defined in `errors.klt`, not `math.klt`. Code that includes `math.klt` without `errors.klt` will not have `EPSILON`.
- **TP Interfaces:** 18 routines are exposed to TP programs via `package.json` `tp-interfaces`. Short names are `mth_*` (e.g., `mth_pow`, `mth_rotx`). These depend on `TPElib`, `pose`, and `registers` at link time.
- **Generic classes are in `include/classes/`**, not in `src/`. They must be expanded via `%class` at the point of use in the caller's source file — they are not compiled standalone.
- **rossum include paths:** `include/` and `include/classes/` and `test/test_types/` are all registered in `package.json`, so dependent packages can reference headers without path prefix.
- **`math.private.klt`** declares the PRNG global `m_seed : INTEGER`. This variable is scoped to the `math.kl` program. Cross-program PRNG sharing is not possible without explicit register I/O.
- **`math__atan2` parameter order warning:** The public API takes `(opposite, adjacent)` (y, x order), but internally calls `ATAN2(adjacent, opposite)` — the FANUC built-in takes `(adjacent, opposite)`. This is correct behavior but easy to confuse when reading the source.
