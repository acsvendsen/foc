# Compact Harmonic BLDC Robot Joint

This repo is an experimental robotic arm joint project.

The core idea is:
- embed a BLDC outrunner inside a harmonic drive
- keep the joint extremely compact
- use ODrive-style FOC control for precise robotic arm motion

This is not a polished product repo. It is an active engineering notebook plus control/tuning toolkit.

## Help Wanted

I am building this in public and I do **not** assume the current architecture is final or optimal.

If you know more than I do about any of these areas, help is welcome:
- BLDC FOC tuning on ODrive-style controllers
- low-vibration settle behavior for robotic joints
- harmonic drive mechanics, preload, compliance, and hysteresis
- motor-side vs output-side encoder architecture
- dual-loop / dual-encoder robot joint control
- quiet low-speed holding without chatter

If you see a wrong path, say it early.

## Project Goal

The target is a compact 6DOF robotic arm where each joint:
- is physically compact
- moves quietly
- reaches commanded joint angles reliably
- avoids vibration and chatter at settle
- can eventually handle real shoulder/elbow-class loads

Short version:
- prototype now
- dead ends called early
- move toward a production-worthy joint architecture

## Current Hardware

Current and recently tested hardware includes:

- Motor:
  - 5065 outrunner
  - about 140 kV
  - 7 pole pairs
- Transmission:
  - 3D-printed harmonic drive prototype
  - several wave-generator / flexspline / pin-bearing variants have been tested
- Controllers:
  - ODESC V4.2 / ODrive-style board
  - MKS ODrive-style board also tested
- Encoder:
  - motor-side magnetic encoder
  - currently MT6701-class ABZ setup in the main path
  - current board/firmware path does **not** expose native dual-encoder support

## Current Engineering Position

The main current lesson is:

- **continuous single-target moves can be quiet**
- **segmented moves with intermediate settled waypoints are what tend to introduce buzzing**

That means the current validated operating mode is:
- one continuous move to the final target
- no intermediate settled stops unless absolutely necessary
- one final soft hold only

This is an important distinction. The problem is not just "small moves" or "large moves". The path shape and settle strategy matter a lot.

## Likely Long-Term Direction

The current controller/encoder setup is good enough for prototyping, but it is unlikely to be the final joint architecture.

The likely long-term direction is:
- keep a motor-side encoder for commutation
- add output-side sensing later for true joint angle
- possibly move to hardware/firmware with cleaner low-level support for that architecture

## Repo Structure

Main working files:

- [common.py](common.py)
  - low-level ODrive-style helpers
  - strict position move path
  - calibration and diagnostics
- [self_calibrate.py](self_calibrate.py)
  - calibration routines and bring-up flow
- [test_motor_setup.py](test_motor_setup.py)
  - higher-level experiments
  - motion profile handling
  - move-to-angle helpers
- `logs/`
  - structured experiment outputs and profile registry
- `3DPrints/`
  - mechanical design iterations for the harmonic-drive experiments

## What Would Be Most Useful From Contributors

Contributions are most useful if they focus on one of these:

1. A better low-level control strategy for quiet settle/hold on ODrive-style FOC
2. Better encoder architecture for this class of compact robotic joint
3. Mechanical insight into harmonic-drive compliance, preload, and repeatability
4. Practical robot-joint control architecture that avoids wasting time on dead ends

## Ground Rules

- Do not assume the current implementation is correct just because it moves.
- Quiet, stable behavior matters more than impressive-looking motion.
- If a path is fundamentally wrong, it is better to say so early than to keep tuning around it.
