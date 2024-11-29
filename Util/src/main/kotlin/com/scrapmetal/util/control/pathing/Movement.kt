package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Vector2d

/**
 * A list of C0 and C1 (position and velocity) continuous splines, with associated heading
 * interpolations and maximum efforts. A single [Movement] starts and stops once: it continually
 * moves through all submovements in succession.
 */
data class Movement(val submovements: List<SubMovement>) {
    private var index = 0

    // TODO: pick a better name for this function
    operator fun invoke(pos: Vector2d): ClosestState {
        val closest = submovements[index](pos)
        if (closest.t == 1.0 && index < submovements.size - 1) {
            index++
        }
        return closest
    }

    fun isLast() = index == submovements.size - 1
}

/**
 * Used for the first [to] in creating a [Movement]
 */
infix fun SplinePoint.to(p: SplinePoint) = Movement(listOf(SubMovement(Spline(
    start = this.position,
    startTangent = this.tangent,
    end = p.position,
    endTangent = p.tangent,
))))

/**
 * Create a [Movement] between two [SplinePoint]s
 */
infix fun Movement.to(p: SplinePoint) = Movement(
    this.submovements + SubMovement(
        Spline(
            this.submovements.last().spline.end,
            this.submovements.last().spline.endTangent,
            p.position,
            p.tangent,
        )
    )
)

infix fun Movement.withHeading(h: HeadingInterpolation) = Movement(
    this.submovements.subList(0, this.submovements.size - 1) + SubMovement(
        this.submovements.last().spline,
        h,
    )
)

infix fun Movement.withEffort(e: Double) = Movement(
    this.submovements.subList(0, this.submovements.size - 1) + SubMovement(
        this.submovements.last().spline,
        this.submovements.last().heading,
        e,
    )
)