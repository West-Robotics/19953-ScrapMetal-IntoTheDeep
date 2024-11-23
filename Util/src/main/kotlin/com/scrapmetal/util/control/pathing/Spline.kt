package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Vector2d
import kotlin.math.pow

/**
 * A cubic Hermite spline
 */
class Spline(
    val start: Vector2d,
    private val v0: Vector2d,
    val end: Vector2d,
    private val v1: Vector2d,
) {
    private val p1: Vector2d = start + v0/3.0
    private val p2: Vector2d = end - v1/3.0
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
    fun invoke(t: Double) = coef[3]*t.pow(3) + coef[2]*t.pow(2) + coef[1]*t + coef[0]

    /**
     * Return the derivative of the path at a parameter [t]
     */
    fun dpdt(t: Double) = coef[3]*3.0*t.pow(2) + coef[2]*2.0*t + coef[1]

    // WARNING: this might have really bad performance
    /**
     * Approximate closest point through an algorithm similar to binary search, but denary
     */
    fun closestPoint(pos: Vector2d): Vector2d {
        tailrec fun closestOf10(lower: Double, upper: Double): Vector2d {
            val range = upper - lower
            fun iToT(i: Int) = lower + range*(0.05 + 0.1*i)
            val distances = DoubleArray(10) { i -> (invoke(iToT(i)) - pos).norm() }
            val closestT = iToT(distances.indices.minBy { distances[it] })
            return if (range >= 0.01) {
                closestOf10(closestT - 0.5*range/10.0, closestT + 0.5*range/10.0)
            } else {
                invoke(closestT)
            }
        }
        return closestOf10(0.0, 1.0)
    }
}