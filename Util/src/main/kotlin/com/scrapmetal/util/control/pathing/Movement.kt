package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Vector2d

/**
 * A list of C0 and C1 (position and velocity) continuous splines, with associated heading
 * interpolations and maximum efforts. A single [Movement] starts and stops once: it continually
 * moves through all submovements in succession.
 */
data class Movement(val submovements: List<SubMovement>) {
    private var index = 0
    val endPoint = submovements.last().curve.end

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
 * Create the first [Spline] [SubMovement] in a [Movement]
 */
infix fun SplinePoint.splineTo(p: SplinePoint) = Movement(listOf(SubMovement(
    Spline(
        start = this.position,
        startTangent = this.tangent,
        end = p.position,
        endTangent = p.tangent,
    )
)))

/**
 * Create the first [Line] [SubMovement] in a [Movement]
 */
infix fun CurvePoint.lineTo(p: CurvePoint) = Movement(listOf(SubMovement(
    Line(
        start = this.position,
        end = p.position,
    )
)))

/**
 * Add a [Spline] [SubMovement] between 2 [SplinePoint]s to a [Movement]
 */
infix fun Movement.splineTo(p: SplinePoint) = Movement(
    this.submovements + SubMovement(
        Spline(
            this.submovements.last().curve.end,
            this.submovements.last().curve.endTangent,
            p.position,
            p.tangent,
        )
    )
)

/**
 * Add a [Line] [SubMovement] between 2 [CurvePoint]s to a [Movement]
 */
infix fun Movement.lineTo(p: CurvePoint) = Movement(
    this.submovements + SubMovement(
        Line(this.submovements.last().curve.end, p.position)
    )
)

infix fun Movement.withHeading(h: HeadingInterpolation) = Movement(
    this.submovements.subList(0, this.submovements.size - 1) + SubMovement(
        this.submovements.last().curve,
        h,
        this.submovements.last().pathSpeed,
    )
)

infix fun Movement.withSpeed(e: Double) = Movement(
    this.submovements.subList(0, this.submovements.size - 1) + SubMovement(
        this.submovements.last().curve,
        this.submovements.last().heading,
        e,
    )
)