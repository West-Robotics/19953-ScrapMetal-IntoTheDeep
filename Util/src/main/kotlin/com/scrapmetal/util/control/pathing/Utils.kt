package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Vector2d
import kotlin.math.pow

// WARNING: this might have really bad performance
/**
 * Approximate closest point through an algorithm similar to binary search, but n-ary.
 *
 * This is done to reduce the chance of not finding a global minimum.
 */
fun closestPoint(pos: Vector2d, spline: Spline): Vector2d {
    fun iToT(i: Int, n: Int, lower: Double, upper: Double) = lower + (upper-lower)*(1.0/n*(0.5 + i))
    tailrec fun closestTOfN(n: Int, lower: Double, upper: Double): Double {
        println("did an efficient iteration")
        val range = upper - lower
        val distances = DoubleArray(n) { i -> (spline(iToT(i, n, lower, upper)) - pos).norm() }
        val closestT = iToT(distances.indices.minBy { distances[it] }, n, lower, upper)
        return if (range > n.toDouble().pow(-1)) {
            closestTOfN(n, closestT - 0.5*range/n, closestT + 0.5*range/n)
        } else {
            closestT
        }
    }
    // compare result from recursive algorithm against start and end points
    return spline(doubleArrayOf(
        closestTOfN(30, 0.0, 1.0),
        0.0,
        1.0
    ).minBy {
        (spline(it) - pos).norm()
    })
}

/**
 * Brute force closest point with a million samples, only use for testing
 */
fun closestPointBruteforce(pos: Vector2d, spline: Spline)
    = spline((1..1_000_000).minBy { i -> (spline(i/1_000_000.0) - pos).norm() } / 1_000_000.0)
