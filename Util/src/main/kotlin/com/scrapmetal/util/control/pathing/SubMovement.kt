package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Vector2d

data class SubMovement(
    val spline: Spline,
    val heading: HeadingInterpolation = Tangent(spline),
    val pathEffort: Double = 1.0,
) {
    /**
     * Return closest pose, derivative, and t
     */
    operator fun invoke(pos: Vector2d) = spline.closestT(pos).let {
        ClosestState(
            Pose2d(spline(it), heading(it)),
            Pose2d(spline.tangentAt(it) * pathEffort, heading.derivative(it)),
            it,
        )
    }
}

data class ClosestState(
    val pose: Pose2d,
    val derivative: Pose2d,
    val t: Double,
)