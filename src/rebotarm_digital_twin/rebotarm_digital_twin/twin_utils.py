#!/usr/bin/env python3

import math

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
MODEL_LIMITS = {
    "dm": (
        (-2.8, -3.14, -3.14, -1.87, -1.57, -3.14),
        (2.8, 0.0, 0.0, 1.57, 1.57, 3.14),
    ),
    "rs": (
        (-2.8, 0.0, 0.0, -1.57, -1.57, -3.14),
        (2.8, 3.14, 3.14, 1.57, 1.57, 3.14),
    ),
}


def joint_limits(model):
    try:
        return MODEL_LIMITS[str(model)]
    except KeyError as exception:
        raise ValueError("model must be 'dm' or 'rs'") from exception


def finite_vector(values):
    return len(values) == 6 and all(
        math.isfinite(float(value)) for value in values
    )


def clamp_vector(values, model="dm"):
    if len(values) != 6:
        raise ValueError("joint vector must contain exactly six values")
    lower_limits, upper_limits = joint_limits(model)
    return tuple(
        max(lower, min(upper, float(value)))
        for value, lower, upper in zip(values, lower_limits, upper_limits)
    )


def within_limits(values, model="dm"):
    lower_limits, upper_limits = joint_limits(model)
    return finite_vector(values) and all(
        lower <= float(value) <= upper
        for value, lower, upper in zip(values, lower_limits, upper_limits)
    )


def rate_limit_vector(previous, target, max_step):
    if len(previous) != 6 or len(target) != 6:
        raise ValueError("joint vector must contain exactly six values")
    if max_step <= 0.0 or not math.isfinite(max_step):
        raise ValueError("max_step must be finite and positive")
    return tuple(
        max(old - max_step, min(old + max_step, new))
        for old, new in zip(previous, target)
    )
