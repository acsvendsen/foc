#!/usr/bin/env python3
"""Shared mounted preload rules for the MKS directional helper path."""

from __future__ import annotations


def select_directional_preload_offset(
    delta_turns: float,
    *,
    min_offset_turns: float = 0.10,
    max_offset_turns: float = 0.15,
    scale: float = 0.30,
) -> float:
    """Choose a preload offset from measured mounted sweep behavior."""
    span = abs(float(delta_turns))
    offset = max(float(min_offset_turns), min(float(max_offset_turns), float(span * float(scale))))
    # Full-span mounted crossovers benefit from a larger preload than the
    # local ±0.25 / ±0.50 moves that established the original clamp.
    if span >= 0.90:
        offset = max(offset, 0.20)
    return float(offset)


def choose_directional_approach(delta_turns: float) -> str:
    delta = float(delta_turns)
    if delta > 0.0:
        return "from_above"
    if delta < 0.0:
        return "from_below"
    return "direct"
