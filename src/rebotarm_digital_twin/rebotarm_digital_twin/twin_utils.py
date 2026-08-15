#!/usr/bin/env python3

import math

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
LOWER_LIMITS = (-2.8, -3.14, -3.14, -1.87, -1.57, -3.14)
UPPER_LIMITS = (2.8, 0.0, 0.0, 1.57, 1.57, 3.14)


def finite_vector(values):
    return len(values) == 6 and all(
        math.isfinite(float(value)) for value in values
    )


def clamp_vector(values):
    if len(values) != 6:
        raise ValueError("joint vector must contain exactly six values")
    return tuple(
        max(lower, min(upper, float(value)))
        for value, lower, upper in zip(values, LOWER_LIMITS, UPPER_LIMITS)
    )


def within_limits(values):
    return finite_vector(values) and all(
        lower <= float(value) <= upper
        for value, lower, upper in zip(values, LOWER_LIMITS, UPPER_LIMITS)
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
