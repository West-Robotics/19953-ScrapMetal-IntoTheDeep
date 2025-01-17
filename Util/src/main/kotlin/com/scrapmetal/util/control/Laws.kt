package com.scrapmetal.util.control

import kotlin.math.abs
import kotlin.math.sign

/**
 * Simple P controller that can also handle wraparound logic for heading etc., in degrees
 */
// TODO: add wraparound logic
fun pControl(gain: Double, reference: Double, state: Double, wraparound: Boolean = false): Double =
    gain * (reference - state).let {
        if (!wraparound || abs(it) < 180.0) it else it - sign(it) * 360.0
    }

/**
 * Return feedforward given a lambda [ff] and the current [state]
 */
fun feedforward(ff: (Double) -> Double, state: Double) = ff(state)

/**
 * Linearly constrain the [input] effort between the minimum [min] and the maximum [max] once
 * outside the [deadzone]. Below [deadzone], effort scales linearly between 0 and [min].
 */
fun constrainEffort(
    input: Double,
    min: Double,
    max: Double,
    deadzone: Double,
) = when {
    abs(input) > deadzone
        -> (max - min) / (max - deadzone) *
           (input - sign(input) * deadzone) +
           sign(input) * min
    else -> input * min / deadzone
}.coerceIn(-max, max)