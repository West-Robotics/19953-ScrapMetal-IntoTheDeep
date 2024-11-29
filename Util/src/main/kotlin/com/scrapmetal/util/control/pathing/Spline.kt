package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d
import kotlin.math.pow

/**
 * A cubic Hermite spline
 */
data class Spline(
    val start: Vector2d,
    val startTangent: Vector2d,
    val end: Vector2d,
    val endTangent: Vector2d,
) {
    private val p1: Vector2d = start + startTangent/3.0
    private val p2: Vector2d = end - endTangent/3.0
    private val coef: Array<Vector2d> = Array(4) {
        when (it) {
            0 ->  start
            1 -> -start*3.0 + p1*3.0
            2 ->  start*3.0 - p1*6.0 + p2*3.0
            3 -> -start     + p1*3.0 - p2*3.0 + end
            else -> Vector2d()
        }
    }
    private val positions: Array<Vector2d> = Array(1001) { t -> invoke(t/1000.0)}

    /**
     * Return a point ([Vector2d]) given a parameter [t]
     */
    operator fun invoke(t: Double) = coef[3]*t.pow(3) + coef[2]*t.pow(2) + coef[1]*t + coef[0]

    /**
     * Return the tangent vector (unit derivative) of the path at a parameter [t]
     */
    fun tangentAt(t: Double) = (coef[3]*3.0*t.pow(2) + coef[2]*2.0*t + coef[1]).unit()

    // WARNING: this might have really bad performance
    /**
     * Approximate closest t through an algorithm similar to binary search, but n-ary.
     *
     * This is done to reduce the chance of not finding a global minimum.
     */
    fun closestT(pos: Vector2d): Double {
        fun iToT(i: Int, n: Int, lower: Double, upper: Double) = lower + (upper-lower)*(1.0/n*(0.5 + i))
        tailrec fun closestTOfN(n: Int, lower: Double, upper: Double): Double {
            val range = upper - lower
            val distances = DoubleArray(n) { i -> (invoke(iToT(i, n, lower, upper)) - pos).norm() }
            val closestT = iToT(distances.indices.minBy { distances[it] }, n, lower, upper)
            return if (range > n.toDouble().pow(-1)) {
                closestTOfN(n, closestT - 0.5*range/n, closestT + 0.5*range/n)
            } else {
                closestT
            }
        }
        // compare result from recursive algorithm against start and end points
        return doubleArrayOf(
            closestTOfN(30, 0.0, 1.0),
            0.0,
            1.0
        ).minBy {
            (invoke(it) - pos).norm()
        }
    }

    fun closestPoint(pos: Vector2d) = invoke(closestT(pos))

    /**
     * Brute force closest point with a million samples, only use for testing
     */
    fun closestPointBruteforce(pos: Vector2d)
            = invoke((1..1_000_000).minBy { i -> (invoke(i/1_000_000.0) - pos).norm() } / 1_000_000.0)
}

/**
 * A convenience class that wraps a spline position and tangent vector, allowing the tangent vector
 * to be specified in polar form
 */
data class SplinePoint(val position: Vector2d, val tangent: Vector2d) {
    constructor(
        x: Double,
        y: Double,
        velocity: Double,
        theta: Double,
    ) : this(Vector2d(x, y), Rotation2d(theta)*Vector2d(velocity, 0.0))
}