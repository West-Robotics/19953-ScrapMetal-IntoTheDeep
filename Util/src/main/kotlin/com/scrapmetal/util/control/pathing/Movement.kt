package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Vector2d

data class Movement(val submovements: List<SubMovement>) {
    private var index = 0

    fun update(pos: Vector2d): Pose2d {
        val state = submovements[index].update(pos)
        if (state.closestT == 1.0 && index < submovements.size - 1) {
            index++
        }
        return state.motion
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
    this.submovements.subList(0, this.submovements.size-1) + SubMovement(
        this.submovements.last().spline,
        h,
    )
)

infix fun Movement.withEffort(e: Double) = Movement(
    this.submovements.subList(0, this.submovements.size-1) + SubMovement(
        this.submovements.last().spline,
        this.submovements.last().heading,
        e,
    )
)
