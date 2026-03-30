# BLDC Motor Side-Project Notes

This note is the motor-focused companion to `docs/embedded_joint_architecture_scorecard.md`.

It captures the practical BLDC/outrunner side-quest decisions for the compact embedded harmonic joint.

## Executive Snapshot

| Topic | Current call | Why |
|---|---|---|
| Keep current stator core first? | `🟢 Yes` | Cheapest way to isolate rotor/winding changes without redesigning the whole motor |
| Lower `kV` in same motor envelope? | `🟢 Yes` | Best electrical improvement for low-speed authority and torque-per-amp |
| Skewed dual-band outer rotor? | `🟢 Yes` | Best magnetic anti-cogging experiment in the current envelope |
| Chase more pole pairs next? | `🔴 No` | Higher redesign cost and less predictable benefit at this diameter |
| Exact original magnet grade required? | `🟠 No` | Geometry, temperature class, and placement matter more for prototype 1 |

## Motor Anatomy Glossary

| Part in the motor | Correct name | What it does |
|---|---|---|
| Gray toothed metal stack | **Stator lamination stack / stator core** | Electrical-steel core that the windings wrap around |
| Individual gray poles | **Stator teeth** | Teeth that carry the coils |
| Circular gray ring joining the teeth | **Back iron / stator yoke** | Magnetic return path in the stator |
| Green molded piece | **Stator insulator / core insulator** | Keeps magnet wire off the sharp laminations |
| Copper wire | **Windings / coils** | Creates the rotating field |
| Red fixed aluminum center carrier | **End bell / stator mount** | Holds the stator, bearing, and shaft |
| Center steel rod | **Shaft** | Mechanical output/input shaft |
| Center bearing | **Bearing** | Supports rotation |
| Black outer shell | **Rotor can / outer rotor housing** | Rotating shell that carries magnets |
| Gray pieces inside the can | **Rotor magnets / magnet ring** | Permanent magnets that interact with the stator |
| Red/yellow/black leads | **Phase leads** | Motor electrical outputs |

## Why The Rotor Can Wall Looks Thick

Practical reasons:

1. **Magnetic back iron**
- The steel rotor can is not just a holder.
- It is the magnetic return path behind the magnets.
- If it is too thin, it can saturate and waste magnet strength.

2. **Mechanical hoop strength**
- The can has to hold the magnets against centrifugal force.
- Thin walls are riskier at speed.

3. **Stiffness and concentricity**
- A thicker can is less likely to deform or run out.

4. **Assembly tolerance margin**
- Cheap motors often use a conservatively thick can because it is easier to manufacture robustly.

So yes: the thick wall has a real purpose. It is both a **magnetic back iron** and a **mechanical containment structure**.

## Magnet Grade Naming

Example: `N42SH`

- `N42` = approximate energy-product class / strength class
- `SH` = **high-temperature grade**

Typical rough meaning:

| Suffix | Typical max operating temperature |
|---|---:|
| none | ~80 C |
| `M` | ~100 C |
| `H` | ~120 C |
| `SH` | ~150 C |
| `UH` | ~180 C |
| `EH` | ~200 C |
| `AH` | ~230 C |

Exact values vary by supplier, but `SH` is the right kind of suffix to look for when the motor will live in a hotter enclosed joint.

## Reuse / Custom Table

### 1. Donor `5065` baseline

| Part | Reuse | Change | Custom |
|---|---|---|---|
| Stator lamination stack | Yes | No | No |
| Green stator insulator | Yes | No | No |
| Shaft | Yes | No | No |
| Bearings | Yes | No | No |
| Red end bell / mount | Yes | No | No |
| Rotor can | Yes | No | No |
| Magnets | Yes | No | No |
| Windings | Yes | No | No |

### 2. Lower-`kV` rewind version

| Part | Reuse | Change | Custom |
|---|---|---|---|
| Stator lamination stack | Yes | No | No |
| Green stator insulator | Yes | Maybe rework if damaged | No |
| Shaft | Yes | No | No |
| Bearings | Yes | No | No |
| Red end bell / mount | Yes | No | No |
| Rotor can | Yes | No | No |
| Magnets | Yes | No | No |
| Windings | No | Yes, rewind | No |

### 3. Skewed-rotor version

| Part | Reuse | Change | Custom |
|---|---|---|---|
| Stator lamination stack | Yes | No | No |
| Green stator insulator | Yes | No | No |
| Shaft | Yes | No | No |
| Bearings | Yes | No | No |
| Red end bell / mount | Usually yes | Maybe spacer change | Maybe |
| Rotor can shell | Maybe | Likely modified | Maybe new |
| Magnet layout | No | Yes | Yes |
| Windings | Yes initially | Optional later | No |

### 4. Lower-`kV` + skewed-rotor version

| Part | Reuse | Change | Custom |
|---|---|---|---|
| Stator lamination stack | Yes | No | No |
| Green stator insulator | Yes | Maybe | No |
| Shaft | Yes | No | No |
| Bearings | Yes | No | No |
| Red end bell / mount | Usually yes | Maybe | Maybe |
| Rotor can shell | Maybe | Likely | Maybe new |
| Magnet layout | No | Yes | Yes |
| Windings | No | Yes | No |

## Skewed Rotor: What Changes

For prototype 1, the clean interpretation is:

- keep the **same stator core**
- keep the **same 12-slot stator**
- keep **7 pole pairs**
- change the **outer rotor magnet layout**

That means:

- replace one long magnet per pole with **two shorter axial magnet segments per pole**
- upper and lower segments have the **same polarity**
- upper and lower bands are given a **small relative angular skew**

Important:

- this usually means **28 physical magnet pieces**
- but still **14 magnetic poles**
- so FOC still uses **7 pole pairs**

### First recommended skew

| Parameter | First try |
|---|---:|
| Total mechanical skew | `2.143°` |
| Lower band | `-1.0715°` |
| Upper band | `+1.0715°` |
| Pole pairs in controller | `7` |

## Torque Hit From Skewing

Small skew is usually a favorable trade.

Approximate 2-step skew penalty for `7` pole pairs:

| Total mechanical skew | Electrical skew | Approx. fundamental torque / back-EMF hit | Practical call |
|---|---:|---:|---|
| `2.1°` | `~15°` | `~0.9%` | `🟢 Strong first prototype` |
| `3.0°` | `~21°` | `~1.7%` | `🟢 Still reasonable` |
| `4.0°` | `~28°` | `~3.0%` | `🟠 Use only if needed` |
| `5.0°` | `~35°` | `~4.7%` | `🔴 Probably too much for first try` |

What you lose by skewing:
- a little torque constant
- a little back-EMF constant
- a little peak torque
- more assembly and balancing sensitivity

What you gain:
- less cogging
- less torque ripple
- smoother low-speed behavior

## Magnet Buying / Prototype Spec Sheet

### Goal
Split each original long rotor magnet into two shorter axial magnets while preserving overall motor behavior as much as possible.

### Measure from the original motor

Record these for **one original magnet**:

| What to measure | Notes |
|---|---|
| Axial length `L` | Full top-to-bottom magnet length |
| Radial thickness `T` | Keep this matched as closely as possible |
| Arc width / chord width `W` | Keep similar coverage around the can |
| Magnet count | Should remain `14` poles overall |
| Rotor can inner diameter | Needed to confirm fit |
| Air gap to stator | Keep unchanged if possible |

### First prototype magnet target

| Spec item | Recommended target |
|---|---|
| Magnet material | NdFeB |
| Grade target | `N42SH` first, `N48H/SH` only if sourcing is easy |
| Coating | NiCuNi or equivalent standard motor magnet coating |
| Radial thickness | Match original magnet thickness |
| Total axial active length | Keep close to original total active length |
| Axial split | Two equal or near-equal segments |
| Center gap | `0.0 mm` to `0.5 mm` preferred, up to `1.0 mm` only if assembly requires it |
| Polarity order | Same alternating `N/S/N/S...` as original |
| Controller pole pairs | Still `7` |

### Example split

If the original magnet axial length is `L`:

- upper segment ≈ `L/2 - small allowance`
- lower segment ≈ `L/2 - small allowance`
- center gap = only what assembly needs

Example:
- original `20 mm`
- upper `9.6 mm`
- lower `9.6 mm`
- center gap `0.8 mm`

### Practical guidance

1. Do **not** obsess over matching the exact original grade first.
2. Match geometry and temperature class first.
3. Keep the axial gap as small as practical.
4. Good placement accuracy matters more than squeezing every last bit of theoretical magnet strength.

## What To Do First

Recommended order:

1. Measure the original rotor magnets
2. Decide whether prototype 1 is:
   - lower `kV` rewind first
   - or skewed-rotor first
3. If skewed rotor first:
   - keep the same stator core
   - keep the same windings initially
   - only change rotor magnet segmentation and indexing
4. Combine lower `kV` and skew only after each helps on its own

## Quick Calls

- Lower `kV` alone will **not** solve cogging.
- Skew alone will **not** solve weak torque authority.
- Lower `kV` + skew is the most coherent next motor path inside the same package.
- More pole pairs are still not the next move.
