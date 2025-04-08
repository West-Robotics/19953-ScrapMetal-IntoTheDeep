package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d

data class SubMovement(
    val curve: Curve,
    val heading: HeadingInterpolation = Tangent(curve),
    val pathSpeed: Double = 1.0,
) {
    /**
     * Return closest pose, derivative, and t
     */
    operator fun invoke(pos: Vector2d) = curve.closestT(pos).let {
        println("heading: ${heading(it)}")
        ClosestState(
            Pose2d(curve(it), heading(it)),
            Pose2d(curve.tangentAt(it) * pathSpeed, Rotation2d(0.0)),
            // Pose2d(curve.tangentAt(it) * pathEffort, heading.derivative(it)), // TODO: add this
            it,
        )
    }
}

data class ClosestState(
    val pose: Pose2d,
    val derivative: Pose2d,
    val t: Double,
)