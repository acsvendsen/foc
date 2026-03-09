# RAG-Style Tuning Assistant (ODrive Project)

This project now includes a structured, log-driven recommendation loop in
`/Users/as/Development/Robot/test_motor_setup.py`.

## Goals

1. Reuse historical run evidence instead of ad-hoc retuning.
2. Track dead ends and failures explicitly.
3. Produce machine-usable profile recommendations and AB candidates.

## Core Functions

1. `build_tuning_knowledge_base(...)`
2. `recommend_motion_profile_from_kb(...)`
3. `run_rag_style_tuning_cycle(...)`
4. `record_tuning_learning_entry(...)`
5. `summarize_learning_ledger(...)`

## Output Artifacts

Generated under `/Users/as/Development/Robot/logs`:

1. `tuning_kb_latest.json`
2. `rag_profile_recommendation_latest.json`
3. `rag_tuning_cycle_latest.json`
4. `learning_ledger_latest.json`

Recommended profiles are also registered into:

1. `stable_diagnostic_profiles_latest.json` (name like `loaded_motion_rag_recommended_v*`)

## Standard Workflow

1. Build recommendation:
```python
cycle = run_rag_style_tuning_cycle(
    log_dir="/Users/as/Development/Robot/logs",
    latest_only=True,
    max_files=200,
    objective="smooth_controlled",
    register_profile=True,
    register_profile_name="loaded_motion_rag_recommended_current",
)
```

2. Validate recommended profile on current hardware:
```python
rep = cycle["recommendation"]
ovr = rep["recommended_profile_overrides"]
res = validate_move_to_angle_sequence(
    targets_deg=(1.0, -1.0, 2.0, -2.0),
    angle_space="gearbox_output",
    axis=odrv0.axis0,
    profile_name="loaded_move_to_angle_micro_i1_20260301",
    profile_overrides=ovr,
    relative_to_current=True,
    startup_checks="guarded",
    pre_move_gate_mode="off",
    smoothness_gate=False,
    fail_on_wrong_direction=True,
    wrong_direction_min_progress_turns=0.001,
    return_to_anchor_at_end=False,
    continue_on_error=True,
    timeout_s=12.0,
    settle_s=0.16,
)
```

3. Log the decision:
```python
record_tuning_learning_entry(
    setup_label="gearbox_mounted_current_test",
    hypothesis="KB recommendation improves smooth controlled motion",
    command_profile=ovr,
    outcome={"summary": res.get("summary", {})},
    decision=("keep" if bool(res.get("summary", {}).get("ok", False)) else "discard"),
    why_it_failed=(None if bool(res.get("summary", {}).get("ok", False)) else "See summary/error rows"),
    next_blocking_check="If failed: compare candidate_b_quiet and candidate_c_authority",
    tags=["rag", "hardware-validation"],
)
```

## Promotion Criteria (Keep vs Discard)

Keep profile only if:

1. `wrong_direction_fail_count == 0`
2. no hard-fault escalation
3. acceptable smoothness/noise for the use case
4. reached ratio meets target for that stage

Discard profile if it regresses any safety gate even when precision improves.
