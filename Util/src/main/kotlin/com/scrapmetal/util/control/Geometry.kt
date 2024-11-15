package com.scrapmetal.util.control

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * A 2-dimensional vector that holds coordinates in a Cartesian plane
 */
data class Vector2d(val x: Double = 0.0, val y: Double = 0.0) {
    infix fun dot(v: Vector2d) = x * v.x + y * v.y
    fun norm() = sqrt(this dot this)
    fun unit() = if (norm() != 0.0) Vector2d(x / norm(), y / norm()) else Vector2d()
    fun normal() = Rotation2d(90.0)*this

    operator fun times(k: Double) = Vector2d(k * x, k * y)
    operator fun div(k: Double) = Vector2d(x / k, y / k)
    operator fun plus(v: Vector2d) = Vector2d(x + v.x, y + v.y)
    operator fun minus(v: Vector2d) = Vector2d(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2d(-x, -y)
}

/**
 * A rotation matrix that can rotate a [Vector2d] by [theta] in degrees
 */
data class Rotation2d(val theta: Degrees = 0.0) {
    operator fun times(v: Vector2d) = Vector2d(
        v.x * cos(theta.toRad()) - v.y * sin(theta.toRad()),
        v.x * sin(theta.toRad()) + v.y * cos(theta.toRad()),
    )
    operator fun times(r: Rotation2d) = Rotation2d(theta + r.theta)
    fun inverse() = Rotation2d(-theta)
}
typealias Degrees = Double
fun Degrees.toRad() = 2 * PI / 360 * this

/**
 * A [position] and [heading] in a 2D plane.
 */
data class Pose2d(val position: Vector2d = Vector2d(), val heading: Rotation2d = Rotation2d()) {
    constructor(x: Double = 0.0, y: Double = 0.0, theta: Double = 0.0) : this()
    operator fun plus(p: Pose2d) = Pose2d(position + p.position, p.heading * heading)
    operator fun minus(p: Pose2d) = Pose2d(position - p.position, p.heading.inverse() * heading)
}