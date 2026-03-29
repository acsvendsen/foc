# Embedded Joint Architecture Scorecard

Last updated: 2026-03-29

## Why This Exists

This document is the stable reference for the current actuator architecture discussion.

It exists to prevent repeating the same debate across chat history and to keep decisions tied to:

- the actual 6DOF arm goal
- the current compact embedded-outrunner harmonic-drive architecture
- the known weak points of the current prototype
- the next design levers worth testing

This is a living engineering note. Update it when the hardware or evidence changes.

## Project Position

The concept is:

- a compact robotic arm joint
- with an outrunner embedded inside the reducer
- currently using a 3D-printed harmonic-drive-style transmission
- with ODrive-style FOC control
- aiming for compact elbow/wrist-friendly packaging

The core product argument is not "better than every pancake motor at everything."

The core product argument is:

- better packaging and integration than standard separated motor + gearbox joints
- while trying to get smoothness and precision good enough to be viable for a serious robotic arm

## Architecture Comparison

| Dimension | Standard Motor + Gearbox | Pancake Robotics Motor + Reducer | Current Embedded Outrunner Prototype | Intended Improved Embedded Design |
|---|---|---|---|---|
| Axial compactness | Worst | Middle | Best | Best |
| Elbow/wrist packaging | Weak | Middle | Best | Best |
| Integration elegance | Low | Middle | Best | Best |
| Off-the-shelf ease | Best | Good | Worst | Worst |
| Low-speed smoothness | Middle | Best | Worst | Target: Middle to Good |
| Cogging risk | Middle | Best | Worst | Target: Middle to Good |
| Thermal ease | Middle | Best | Hard | Hard |
| Control/tuning difficulty | Middle | Easiest | Hardest | Hard |
| Product differentiation | Low | Middle | Best | Best |
| Precision potential | Middle | Best | Weak today | Plausibly competitive with standard |
| Chance of compact arm advantage | Middle | Middle | Best | Best |

## What This Means

The current prototype is weak on:

- cogging / low-speed roughness
- tiny-move smoothness
- jitter near settle
- narrow usable low-speed band

The intended improved design is explicitly trying to move those rows upward while keeping the packaging advantage.

That means the relevant comparison is:

- beat standard form-factor joints on compactness and integration
- do not lose too badly to pancake actuators on smoothness and control quality

If it cannot do both well enough, the path is a dead end.

## Main Improvement Levers

### 1. Better control

Best for:

- reducing self-inflicted jitter
- improving capture/settle behavior
- exposing true plant limits

Does not directly solve:

- magnetic cogging
- bad motor geometry

### 2. Lower kV in the same motor geometry

Best for:

- higher torque per amp
- less current for the same torque
- better low-speed authority
- less violent breakaway assistance

Does not directly solve:

- cogging torque caused by slot/pole geometry

### 3. Skewed dual-band outer rotor

Best for:

- cogging reduction
- torque ripple reduction
- smoother low-speed feel

This is the most relevant magnetic-geometry experiment for the compact embedded architecture.

It should be understood as:

- same 14 magnets
- same 7 pole pairs
- two axial magnet bands
- small relative skew

It is not a way to turn 7 pole pairs into 14 pole pairs.

### 4. More pole pairs

Possible long-term redesign path, but not the first safe lever.

Why it ranks later:

- harder to fit cleanly at this diameter
- much larger magnetic redesign
- easier to get wrong
- results are less predictable than lower kV + skew

## Decision Matrix

| Lever | Expected gain on current problems | Effort | Risk | Keeps current architecture intact? | Why it matters |
|---|---|---|---|---|---|
| Better control | Medium to High | Medium | Low to Medium | Yes | Reduces self-inflicted jitter and reveals plant truth |
| Lower kV in same motor geometry | High | Medium to High | Medium | Yes | Improves torque-per-amp and low-speed authority |
| Skewed dual-band outer rotor | High | High | Medium to High | Yes, mostly | Best direct attack on cogging without increasing diameter |
| More pole pairs | Uncertain to Medium | Very High | High | Not really | Full redesign path, easy to waste time on |
| Increase gear ratio | Low to Medium | High | Medium to High | Maybe | Helps multiplication but can worsen compliance/hysteresis sensitivity |
| Standard form-factor redesign | High | Very High | Medium | No | Easier engineering path, weakens core packaging advantage |
| Pancake-style redesign | High to Very High | Very High | Medium | No | Strong smoothness path, abandons the compact embedded differentiator |

## Practical Ranking

### Best immediate engineering sequence

1. Better control
2. Lower kV in the same motor envelope
3. Skewed dual-band rotor
4. More pole pairs later, only if still justified

### Best direct attack on the ugly low-speed feel

1. Skewed dual-band rotor
2. Lower kV
3. Better control
4. More pole pairs

## Milestone Table

These are the pass/fail gates for deciding whether the embedded-outrunner architecture is still worth pushing.

### Stage 1: Control Truth

Goal:

- prove the controller is no longer the main liar

| Metric | Pass | Borderline | Fail |
|---|---|---|---|
| Output encoder trust | output motion consistently matches observed joint motion | mostly matches, some ambiguity | cannot trust output motion reports |
| Recovery from bad runs | axis returns cleanly to idle/ready | sometimes needs manual recovery | frequently ends in bad state |
| Small-move jitter from control alone | low and understandable | present but bounded | controller injects obvious chatter |
| Manual test repeatability | same test gives similar outcome repeatedly | some drift | results too inconsistent to reason from |

### Stage 2: Low-Speed Usability

Goal:

- prove the joint can move slowly without feeling fundamentally ugly

| Metric | Pass | Borderline | Fail |
|---|---|---|---|
| Breakaway floor | low and repeatable | repeatable but still a bit high | erratic or too high |
| Usable low-speed band | clearly present and repeatable | narrow and fragile | basically absent |
| Audible/mechanical roughness | modest | noticeable | clearly bad |
| Assisted low-speed runs | clearly better than raw | only slightly better | not meaningfully better |

### Stage 3: Small Output Move Quality

Goal:

- prove the joint can do precision-relevant micro motions without nonsense

| Metric | Pass | Borderline | Fail |
|---|---|---|---|
| Tiny commanded output moves | smooth and consistent | works but ugly | unreliable |
| Retained error after small move | small and repeatable | moderate but manageable | large / inconsistent |
| Near-target chatter | absent or minor | noticeable | obvious and unacceptable |
| Return-to-idle behavior | clean | some coast or softness | unstable or buzzing |

### Stage 4: Hold Quality

Goal:

- prove the joint can stay where it should without sounding or behaving wrong

| Metric | Pass | Borderline | Fail |
|---|---|---|---|
| Quiet hold near target | yes | sometimes | no |
| Output drift in hold | small | noticeable | large |
| Buzzing / dithering | absent | occasional | common |
| Recovery from disturbance | sane | usable but soft | unstable or ugly |

### Stage 5: Thermal Sanity

Goal:

- prove the compact embedded motor is not cooking itself

| Metric | Pass | Borderline | Fail |
|---|---|---|---|
| Motor heating in repeated small-move tests | acceptable | warm but maybe manageable | too hot too fast |
| Hold-current thermal burden | acceptable | concerning | unacceptable |
| Enclosed-joint thermal behavior | stable | constrained | deal-breaker |
| Performance drift with temperature | modest | noticeable | severe |

### Stage 6: Multi-Joint Readiness

Goal:

- prove this can scale beyond a single impressive prototype

| Metric | Pass | Borderline | Fail |
|---|---|---|---|
| Repeatability across runs | strong | okay | weak |
| Repeatability across multiple builds | plausible | unknown | poor |
| Tuning burden per joint | manageable | high | excessive |
| Precision behavior with realistic arm loads | acceptable | uncertain | poor |

## Go / No-Go Rule

Keep pushing the embedded architecture if, after:

1. better control
2. lower kV
3. skewed rotor work

the joint reaches:

- smooth slow motion
- repeatable small moves
- no obvious chatter near settle
- sane thermal behavior
- trustworthy output-encoder-based measurements

Call the path a dead end if those changes still leave:

- ugly low-speed feel
- persistent near-target jitter
- poor retained precision
- or bad thermal behavior inside the compact package

## Skewed Outer Rotor Notes

For the current `12-slot / 14-pole` outer rotor concept:

- keep `14` magnets total
- keep `7` pole pairs
- split the rotor magnet set into two axial bands
- apply a small relative skew between the two bands

Recommended first prototype:

- total skew: about `2.143° mechanical`
- lower band: `-1.0715°`
- upper band: `+1.0715°`

This is a cogging / torque-ripple reduction experiment.

It should make FOC harder only if manufactured badly. In principle it remains:

- same 3-phase motor
- same 7 pole pairs
- same overall FOC class

## Patent / Prior-Art References

### Embedded outer-rotor harmonic joint references

1. [US20210197404A1 - Robot joint and robot having the same](https://patents.google.com/patent/US20210197404A1)
2. [CN111113477B - Robot joint structure and robot](https://patents.google.com/patent/CN111113477B/en)
3. [CN114810951A - Brushless outer rotor actuator for robots with built-in reducer](https://patents.google.com/patent/CN114810951A/en)
4. [US8757029B2 - Strain wave gearing and robotic arm](https://patents.google.com/patent/US8757029B2/en)

### Skewed rotor references

1. [US20040207280A1 - Brushless DC motor with stepped skewed rotor](https://patents.google.com/patent/US20040207280A1/en)
2. [US20080024028A1 - Permanent magnet electric motor](https://patents.google.com/patent/US20080024028A1/en)
3. [WO2013018697A1 - Permanent magnet type synchronous motor](https://patents.google.com/patent/WO2013018697A1/ja)
4. [Altair FluxMotor - Step skew technology](https://2024.help.altair.com/2024.1/FluxMotor/topics/step_skew_technology_available.htm)

Important:

- There is strong prior art for the embedded outer-rotor harmonic joint concept.
- There is strong prior art for skewed/segmented rotor sections.
- There is not yet a single exact reference identified that cleanly combines both ideas in the same disclosure.

## Winding / Rewind Terminology

### The green part on the stator

Names:

- stator core insulator
- insulation frame
- tooth insulator
- molded stator insulator

Purpose:

- isolates the magnet wire from the lamination edges
- protects tooth tops and slot entry points

For a one-off rewind, the practical equivalent is usually:

- slot liner insulation
- Nomex / NMN / DMD insulation paper
- end-turn tape / lacing

### The "frame" for wrapping coils

The useful names are:

1. hand shuttle
2. winding shuttle
3. stator holder
4. winding mandrel
5. needle winding nozzle or stator winding needle for industrial equipment

The compact-motor rewind path is usually:

- custom stator holder
- small hand shuttle
- insulation liners and lead sleeving

not a generic bobbin.

## Supply / Material References

These are reference links, not endorsements.

### Magnet wire

Use:

- round enamelled copper magnet wire
- Class 200 minimum
- preferably polyester-imide with polyamide-imide overcoat
- NEMA MW 35-C or equivalent

Links:

1. [Remington 200C 24 AWG polyamide-imide magnet wire](https://www.remingtonindustries.com/magnet-wire/magnet-wire-200c-24-awg-polyamideimide-8-spool-sizes/)
2. [Remington 200C product family](https://www.remingtonindustries.com/magnet-wire/200-c-polyamideimide-1/)
3. [Remington 22 AWG magnet wire page, including 200C kit references](https://www.remingtonindustries.com/magnet-wire/magnet-wire-by-awg/magnet-wire-22-awg/)
4. [MWS multifilar magnet wire](https://mwswire.com/specialty-wire/multifilar-magnet-wire/)

### Winding tooling

1. [Stator winding needles / nozzles](https://www.eurotubes-uk.com/product-category/stator-winding-needle-winding-nozzle-motor-winding-tubes-stators/)

### Insulation / lacing

1. [Nomex lacing tape - Breyden](https://www.breydenproducts.com/shop/electrical-lacing-tapes-cords-and-ropes/nomex-lacing-tape)
2. [Flat braided Nomex lacing tape - Western Filament](https://wfilament.com/product/flat-braided-nomex-lacing-tape/)
3. [Nomex lacing tape - Silmid](https://www.silmid.com/us/tapes/specialty-tapes/Western-Filament-Nomex-Lacing-Tape-Natural-N-Finish-Liquid-Nylon-Size-3-500-Yard-Roll-in-a-Pouch-A-A-52084-Type-V/)

### Motor insulation / stator insulator references

1. [Motor insulation frames / molded stator insulation examples](https://motorpartsupplier.com/motor-insulation/)

## Current Rewind Direction

Current working recommendation:

- do not assume lower kV alone will solve cogging
- do not assume skew alone will solve torque authority
- treat lower kV and anti-cogging geometry as complementary

Most defensible current sequence:

1. keep improving control until it stops revealing new truth
2. move toward lower kV in the same motor envelope
3. prototype a skewed dual-band outer rotor
4. do not chase more pole pairs yet

## Update Rules

Update this document when:

- motor geometry changes
- pole count changes
- kV target changes
- skew experiment results arrive
- thermal data changes the viability picture
- multi-joint scaling evidence changes the go/no-go call
