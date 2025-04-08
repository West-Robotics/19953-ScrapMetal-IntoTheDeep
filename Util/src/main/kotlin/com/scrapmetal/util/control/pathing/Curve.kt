package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d

interface Curve {
    val start: Vector2d
    val end: Vector2d
    val endTangent: Vector2d
    /**
     * Return a point ([Vector2d]) given a parameter [t]
     */
    operator fun invoke(t: Double): Vector2d

    /**
     * Return the unit tangent vector (normalized derivative) of the path at a parameter [t]
     */
    fun tangentAt(t: Double): Vector2d

    fun closestT(pos: Vector2d): Double
}

interface CurvePoint {
    val position: Vector2d
}