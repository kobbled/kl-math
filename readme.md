# math

Extended math functions and algorithms for FANUC Karel.

## Overview

Karel's built-in math library is minimal: trigonometry, ABS, SQRT, and little else. `math` fills the gaps needed by robot kinematics, geometry, and path-planning code. It provides:

- Exponentiation and logarithms (pow, log10, log2)
- Rounding (floor, ceil, round, decimal formatting)
- Arc/angle conversions
- Hyperbolic functions
- A seeded linear-congruential PRNG (rand, rand_range, rand_position, rand_vector)
- Array statistics (min, max, sum, average) and quicksort
- Vector algebra (norm, projection, Manhattan distance)
- Intrinsic pose rotation and translation wrappers around FANUC's `:` operator
- Two GPP-based generic sorting class templates for custom types (`arraysort.klc`, `pathsort.klc`)

Almost every module above Layer 1 in Ka-Boost depends on `math`.

---

## Files

| File | Purpose |
|---|---|
| `src/math.kl` | All implementation — single compilation unit |
| `include/math.klt` | Public constants (M_PI, M_E, M_RAD2DEG, MAX_INT, etc.) |
| `include/math.klh` | Public function declarations (47 routines) |
| `include/math.builtins.klh` | Declarations for built-in wrappers (9 routines) |
| `include/math.private.klh` | Private quicksort helpers |
| `include/math.private.klt` | PRNG parameters (A=25173, C=13849, M=65536) and `m_seed` global |
| `include/classes/arraysort.klc` | Generic quicksort/bubblesort/dedup for `ARRAY[*]` |
| `include/classes/arraysort.klh` | Declarations for arraysort |
| `include/classes/pathsort.klc` | Same algorithms for Karel `PATH` |
| `include/classes/pathsort.klh` | Declarations for pathsort |
| `test/test_math.kl` | 18 tests, 43 assertions |
| `test/test_pth_srt.kl` | 6 tests for arraysort/pathsort |
| `test/test_ls_math.ls` | TP cross-over test for mth_* interface routines |
| `test/test_types/testarr.klt` | arraysort config for `t_TST_ARR` |
| `test/test_types/testpath.klt` | pathsort config for `STD_PTH_NODE` |

---

## API Reference

### Constants (`math.klt`)

```karel
-- Angle / trig
M_PI     = 3.14159265358979323846
M_2PI    = 6.283185307
M_PI_2   = 1.57079632679489661923
M_RAD2DEG = 57.29577951       -- multiply radians to get degrees
M_DEG2RAD = 0.0174532925      -- multiply degrees to get radians

-- Algebra
M_E      = 2.71828182845...
M_SQRT2  = 1.41421356...
M_LN2    = 0.69314718...
M_LN10   = 2.30258509...
M_LOG2E  = 1.44269504...
M_LOG10E = 0.43429448...

-- Limits
MAX_INT  = 32767
MAX_BYTE = 255
MAX_RAND = 13849
```

> **Note:** `EPSILON = 0.0001` (the near-zero threshold used throughout Ka-Boost) is defined in `errors.klt`, not here. Include `errors.klt` when you need it.

---

### Built-in Wrappers

Thin wrappers over FANUC Karel built-ins. The important ones with non-trivial behavior:

```karel
-- Safe square root (returns 0 for near-zero input)
math__sqrt(val : REAL) : REAL

-- atan2 with zero guard
-- Public parameter order is (opposite, adjacent) i.e. (y, x)
-- This is the reverse of FANUC's ATAN2(adjacent, opposite) — the wrapper handles the swap
math__atan2(opposite : REAL; adjacent : REAL) : REAL

-- atan from an XYZWPR — returns ATAN2(pose.x, pose.y), guards both-zero
math__atan_pos(pose : XYZWPR; grp_no : INTEGER) : REAL
```

Angles for `sin`, `cos`, `tan` are in **degrees** (Karel convention).

---

### Logarithmic / Exponent

```karel
-- Raise value to a power (value must be > 0; uses EXP(exp * LN(value)))
math__pow(value : REAL; exponent : REAL) : REAL

-- Base-10 and base-2 logs
math__log10(num : REAL) : REAL
math__log2(num : REAL) : REAL
```

---

### Rounding / Formatting

```karel
math__floor(num : REAL) : INTEGER     -- largest integer <= num
math__ceil(num : REAL) : INTEGER      -- smallest integer >= num
math__round(num : REAL) : INTEGER     -- nearest integer (>=0.5 rounds up)

-- Format to N decimal places and return as REAL
-- Handles edge cases in CNV_REAL_STR (leading zeros, minus-dot notation)
math__decimal(num : REAL; digits : INTEGER) : REAL
```

---

### Arc / Angle

```karel
-- Arc length from central angle (degrees) and radius
math__arclength(degrees : REAL; radius : REAL) : REAL

-- Central angle (degrees) from arc length and radius
math__arcangle(length : REAL; radius : REAL) : REAL

-- Normalize an angle to [0, 360) -- only adds/subtracts 360 once
math__map_to_360(angle : REAL) : REAL
```

---

### Hyperbolic

```karel
math__cosh(x : REAL) : REAL     -- 0.5 * (e^x + e^-x)
math__sinh(x : REAL) : REAL     -- 0.5 * (e^x - e^-x)
```

---

### Miscellaneous

```karel
-- Returns 2^(num-1) as INTEGER
math__bitmask(num : INTEGER) : INTEGER
```

---

### Linear Mapping

```karel
-- Map value x from [in_min, in_max] to [out_min, out_max]
math__map_real(x, in_min, in_max, out_min, out_max : REAL) : REAL
math__map_int(x, in_min, in_max : REAL; out_min, out_max : INTEGER) : INTEGER
```

---

### PRNG (Linear Congruential Generator)

The generator uses `next = (seed x 25173 + 13849) MOD 65536`. State is stored in the global `m_seed` variable in `math.kl`.

```karel
-- Seed the generator (use GET_TIME or GET_USEC_TIM for non-deterministic runs)
math__srand(seed : INTEGER)

math__rand() : REAL                           -- uniform [0.0, 1.0)
math__rand_range(min, max : REAL) : REAL      -- uniform [min, max)
math__rand_int() : INTEGER                    -- uniform [0, MAX_INT)
math__rand_color() : INTEGER                  -- uniform [0, 255)

-- Random XYZWPR in a 300x300x300 mm box, WPR in [0, 90] degrees each axis
-- Re-seeds internally per axis using (seed * axis_index)
math__rand_position(seed : INTEGER) : XYZWPR

-- Random VECTOR in 300x300x300 mm box; re-seeds per dimension
math__rand_vector(seed : INTEGER) : VECTOR

-- Fill entire array with random reals; re-seeds per element
math__rand_rarr(seed : INTEGER; out_arr : ARRAY[*] OF REAL)
```

> **Warning:** `rand_position`, `rand_vector`, and `rand_rarr` call `math__srand` internally, changing the global PRNG state. If you need a controlled sequence after calling these, call `math__srand` again.

---

### Array Statistics

```karel
math__max_real(numbers : ARRAY[*] OF REAL) : REAL
math__max_real_index(numbers : ARRAY[*] OF REAL) : INTEGER
math__min_real(numbers : ARRAY[*] OF REAL) : REAL
math__min_real_index(numbers : ARRAY[*] OF REAL) : INTEGER

math__max_int(numbers : ARRAY[*] OF INTEGER) : INTEGER
math__max_int_index(numbers : ARRAY[*] OF INTEGER) : INTEGER
math__min_int(numbers : ARRAY[*] OF INTEGER) : INTEGER
math__min_int_index(numbers : ARRAY[*] OF INTEGER) : INTEGER

-- Both skip UNINIT() elements; average_real returns 0 if all elements are uninit
math__sum_real(numbers : ARRAY[*] OF REAL) : REAL
math__average_real(numbers : ARRAY[*] OF REAL) : REAL
```

---

### Sorting (Built-in)

```karel
-- In-place ascending quicksort
math__quicksort_real(arr : ARRAY[*] OF REAL)
math__quicksort_int(arr : ARRAY[*] OF INTEGER)
```

For sorting custom types or `PATH`-based collections, see the generic class templates below.

---

### Vector Math

```karel
-- Euclidean norm (magnitude)
math__norm(v : VECTOR) : REAL

-- Squared norm -- prefer over norm for comparisons (avoids SQRT)
math__norm2(v : VECTOR) : REAL

-- Vector projection onto axis ax
math__proj(v, ax : VECTOR) : VECTOR
math__proj_orthoganal(v, ax : VECTOR) : VECTOR   -- component of v perpendicular to ax
math__proj_length(v, ax : VECTOR) : REAL          -- scalar length of projection

-- Component-wise mean; skips UNINIT elements
math__average_vector(arrv : ARRAY[*] OF VECTOR) : VECTOR

-- L1 (Manhattan) distance
math__manhattan_dist(start_v, end_v : VECTOR) : REAL
```

---

### Pose Transformations

These wrap the FANUC `:` operator (frame/pose composition — homogeneous transform multiplication).

**Translation:**
```karel
-- Translate pose in its own local frame by the given vector
math__translate(pos : XYZWPR; amount : VECTOR) : XYZWPR
-- Equivalent to: pos : {amount.x, amount.y, amount.z, 0, 0, 0}
```

**Intrinsic Rotations** (rotate around the pose's own current axes, not world axes):
```karel
math__rotx(pos : XYZWPR; angle : REAL) : XYZWPR  -- rotate W (around X)
math__roty(pos : XYZWPR; angle : REAL) : XYZWPR  -- rotate P (around Y)
math__rotz(pos : XYZWPR; angle : REAL) : XYZWPR  -- rotate R (around Z)
```

**Vector Rotations** (rotates the XYZ components; WPR of result is discarded):
```karel
math__rotx_vec(pos : VECTOR; angle : REAL) : VECTOR
math__roty_vec(pos : VECTOR; angle : REAL) : VECTOR
math__rotz_vec(pos : VECTOR; angle : REAL) : VECTOR
```

> **Important:** The `:` operator applies rotations in the **local (tool) frame**, not the world frame. For world-frame composition, use `matpose` (4x4 matrix multiplication).

---

### Generic Sorting Classes

#### `arraysort.klc` — Sort `ARRAY[*]` of any type

Expand this class with GPP to sort arrays of a custom type.

**Step 1: Create a `.klt` config file** (e.g., `mysort.klt`):
```karel
-- Element type
%define ARRAYTYPE  t_MY_TYPE
-- Equality tolerance (used in dedup)
%define ARRAYEPSILON  0.001

-- Extract a REAL sort key from one element
ROUTINE callback(elem : t_MY_TYPE) : REAL
BEGIN
  RETURN(elem.priority)
END callback

-- Greater-or-less-than comparator
ROUTINE glt(type1, type2 : t_MY_TYPE; comparator : BOOLEAN) : BOOLEAN
BEGIN
  IF comparator THEN RETURN(callback(type1) > callback(type2))
  ELSE RETURN(callback(type1) < callback(type2))
  ENDIF
END glt

-- Equality check
ROUTINE eq(type1, type2 : t_MY_TYPE) : BOOLEAN
BEGIN
  RETURN(ABS(callback(type1) - callback(type2)) < ARRAYEPSILON)
END eq
```

**Step 2: Expand the class** (in your `.kl` source):
```karel
%class mysort('arraysort.klc', 'arraysort.klh', 'mysort.klt')
```

**Step 3: Call the generated methods:**
```karel
-- Sort ascending (comparator=FALSE) or descending (TRUE)
mysort__quicksort(my_array, 1, ARRAY_LEN(my_array), FALSE)
mysort__bubblesort(my_array, 1, ARRAY_LEN(my_array), FALSE)

-- Remove consecutive equal elements (sort first); returns new count
new_len = mysort__remove_duplicates(my_array, 1, ARRAY_LEN(my_array))

-- Full deduplication (sort first, then deduplicate)
mysort__quicksort(my_array, 1, n, FALSE)
new_len = mysort__keep_unique(my_array, 1, n)
```

#### `pathsort.klc` — Sort Karel `PATH` of any node type

Identical API but operates on a Karel `PATH` (linked list). Config additionally requires `LISTHEADER` (PATH header type) and `LISTTYPE` (node type). Methods take `list : PATH` instead of `ARRAY[*]`.

---

## Common Patterns

### 1. Normalize a vector before using it as a direction or frame axis

Always guard against zero-length vectors before dividing — a null vector will cause a runtime error.

```karel
n = math__norm(my_vec)
IF n > EPSILON THEN
  unit_vec = my_vec / n
ELSE
  -- handle degenerate case
ENDIF
```

`math__norm2` avoids SQRT when you only need to compare magnitudes.

---

### 2. Cylindrical or polar coordinate conversion via rotation chains

Rotations compose right-to-left: the innermost call is applied first.

```karel
-- Convert (theta_deg, r, z) cylindrical point to Cartesian, rotating around Z axis
v.x = 0 ; v.y = r ; v.z = z
cart_pos = math__translate(math__rotz(origin_frame, theta_deg), v)

-- Two-axis rotation for polar (theta around Z, phi around Y after that)
v.x = 0 ; v.y = 0 ; v.z = r
cart_pos = math__translate(math__roty(math__rotz(origin_frame, theta_deg), phi_deg), v)
```

---

### 3. Perpendicular direction in 2D

For 2D geometry (z=0 vectors), `math__rotz_vec` is the standard way to get a perpendicular.

```karel
-- Rotate 90 degrees to get the perpendicular direction
perp = math__rotz_vec(direction_vec, 90)

-- Rotate -90 degrees for the opposite perpendicular
other_perp = math__rotz_vec(direction_vec, -90)
```

---

### 4. Seeded PRNG for test data

```karel
-- Non-deterministic: seed from clock
GET_TIME(tme)
math__srand(tme)
r = math__rand_range(0.0, 100.0)

-- Deterministic: fixed seed for repeatable tests
math__srand(42)
FOR i = 1 TO 10 DO
  test_arr[i] = math__rand
ENDFOR
```

---

### 5. Sort a custom type with the generic sort class

See the config example under the Generic Sorting Classes section above.

```karel
-- After %class expansion in your source file:
mysort__quicksort(items, 1, item_count, FALSE)     -- sort ascending
item_count = mysort__keep_unique(items, 1, item_count)  -- remove duplicates
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---|---|---|
| `math__pow(0, exp)` or negative base | Runtime error — LN(0) is undefined | Guard with `IF base > EPSILON` |
| Dividing by `math__norm` without a zero-check | Runtime division-by-zero | Check `math__norm(v) > EPSILON` before dividing |
| Using `math__rotx/y/z` expecting world-frame rotation | Pose drifts incorrectly with chained transforms | These are intrinsic (local-frame). Use `matpose` + matrix multiply for world-frame composition |
| Calling `math__rand` before `math__srand` | All runs produce the same sequence starting from seed=0 | Seed with `GET_TIME` or `GET_USEC_TIM` before first use |
| Expecting `math__map_to_360` to wrap any angle | Returns wrong values for angles outside (-360, 720) | Only adds/subtracts 360 once; apply in a WHILE loop for arbitrary angles |
| Forgetting `callback`, `glt`, `eq` in `.klt` config for sort class | GPP compile error on `%class` expansion | All three routines are required |
| Calling `rand_position` or `rand_vector` mid-sequence | Global `m_seed` is unpredictably modified | Call `math__srand` again after these functions if you need a controlled sequence |

---

## Build Flow

`math` compiles as a single program (`math.kl -> math.pc`). It is a dependency of nearly everything in the Ka-Boost library stack; rossum resolves it automatically.

```shell
rossum .. -w -o     # generates build.ninja with math as a transitive dep
ninja               # compiles math.kl -> math.pc plus all dependents
kpush               # deploys to controller
```

The generic class files (`arraysort.klc`, `pathsort.klc`) are **not compiled standalone** — they are expanded inline by GPP at the point of use via `%class`. The include paths are registered in `package.json` so dependent packages can reference them without path prefixes.

For full build instructions see the [Ka-Boost readme](../../readme.md).
