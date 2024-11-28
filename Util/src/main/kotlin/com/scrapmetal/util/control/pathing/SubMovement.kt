package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Vector2d

data class SubMovement(
    val spline: Spline,
    val heading: HeadingInterpolation = Tangent(spline),
    val effort: Double = 1.0,
) {
    /**
     * Return drivetrain vector, reference heading and closest T
     */
    fun update(pos: Vector2d): MovementState {
        return spline.closestT(pos).let {
            MovementState(
                Pose2d(spline(it)*effort, heading(it)),
                it
            )
        }
    }
}

data class MovementState(
    val motion: Pose2d,
    val closestT: Double,
)