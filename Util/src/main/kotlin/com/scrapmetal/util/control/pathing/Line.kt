package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.toDegrees
import kotlin.math.atan2
import kotlin.math.pow

data class Line(
    override val start: Vector2d,
    override val end: Vector2d,
) : Curve {
    override val endTangent = Rotation2d((end - start).let { atan2( it.y, it.x ).toDegrees() }) * Vector2d(1.0, 0.0)

    override operator fun invoke(t: Double) = start * (1 - t) + end * t

    override fun tangentAt(t: Double) = endTangent

    // WARNING: this might have really bad performance
    // TODO: use analytical solution
    /**
     * Approximate closest t through an algorithm similar to binary search, but n-ary.
     *
     * This is done to reduce the chance of not finding a global minimum.
     */
    override fun closestT(pos: Vector2d): Double {
        fun iToT(i: Int, n: Int, lower: Double, upper: Double) =
            lower + (upper - lower) * (1.0 / n * (0.5+i))
        tailrec fun closestTOfN(n: Int, lower: Double, upper: Double): Double {
            val range = upper - lower
            val distances = DoubleArray(n) { i -> (invoke(iToT(i, n, lower, upper)) - pos).norm }
            val closestT = iToT(distances.indices.minBy { distances[it] }, n, lower, upper)
            return if (range > n.toDouble().pow(-1)) {
                closestTOfN(n, closestT - 0.5 * range / n, closestT + 0.5 * range / n)
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
            (invoke(it) - pos).norm
        }
    }
}

/**
 * A convenience class for path construction that wraps a line position
 */
data class LinePoint(override val position: Vector2d) : CurvePoint {
    constructor(
        x: Double,
        y: Double,
    ) : this(Vector2d(x, y))
}