package com.scrapmetal.util.control

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * Simple P controller
 */
fun pControl(gain: Double, reference: Double, state: Double) = gain * (reference - state)

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
): Double {
    return when {
        abs(input) > deadzone
            -> (max - min) / (max - deadzone) *
               (input - sign(input) * deadzone) +
               sign(input) * min
        else -> input * min / deadzone
    }.coerceIn(-max, max)
}

// WARNING: untested
// TODO: test that it works
// TODO: measure performance on chub
//  run a bunch and time
/**
 * Returns position, velocity, and acceleration information for a trapezoidal motion profile
 * given movement [constraints] and a time [t]
 */
fun motionProfile(constraints: MPConstraints, t: Double): MPState {
    with (constraints) {
        // signed
        val s = end - start
        // unsigned
        val vMax = min(vLimit, accel * sqrt( abs(s) / (0.5 * accel * (1 + accel/decel))))
        val tA = vMax / accel
        val tD = tA * accel / decel
        val tV = (abs(s) - 0.5 * accel * tA.pow(2) - 0.5 * decel * tD.pow(2)) / abs(vMax)

        // all signed
        val a = accel * sign(s)
        val v = vMax * sign(s)
        val d = decel * sign(s)

        return MPState(
            start +
                   0.5 * a * min(max(t, 0.0), tA).pow(2) +
                   v * min(max(t-tA, 0.0), tV) +
                   v * min(max(t-tA-tV, 0.0), tD) -
                       0.5 * d * min(max(t-tA-tV, 0.0), tD).pow(2),
            listOf(a * t, v, v - d * (t-tA-tV)).minBy { abs(it) },
            when (t) {
                in 0.0..tA -> a
                in tA..tA+tV -> 0.0
                in tA+tV..tA+tV+tD -> -d
                else -> 0.0
            },
        )
    }
}

/**
 * Movement constraints for an asymmetric trapezoidal motion profile.
 * [accel], [decel], and [vLimit] should always be positive even for reversed movement.
 */
data class MPConstraints(
    val start: Double,
    val end: Double,
    val accel: Double,
    val decel: Double,
    val vLimit: Double,
)

/**
 * Data returned by [motionProfile] giving current position, velocity, and acceleration
 */
data class MPState(
    val s: Double,
    val v: Double,
    val a: Double,
)