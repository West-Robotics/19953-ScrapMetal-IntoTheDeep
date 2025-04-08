package com.scrapmetal.util.control

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * A 2-dimensional vector that holds coordinates in a Cartesian plane
 */
data class Vector2d(val x: Double, val y: Double) {
    infix fun dot(v: Vector2d) = x * v.x + y * v.y
    val norm get() = sqrt(this dot this)
    val unit get() = if (norm != 0.0) this / norm else Vector2d(1.0, 0.0)
    val normal get() = Rotation2d(90.0)*this

    operator fun times(k: Double) = Vector2d(k * x, k * y)
    operator fun div(k: Double) = Vector2d(x / k, y / k)
    operator fun plus(v: Vector2d) = Vector2d(x + v.x, y + v.y)
    operator fun minus(v: Vector2d) = Vector2d(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2d(-x, -y)
}

/**
 * A rotation matrix for rotating a [Vector2d] by [theta] in degrees
 *
 * This can also be used for representing headings
 */
data class Rotation2d(val theta: Degrees = 0.0) {
    operator fun times(v: Vector2d) = Vector2d(
        v.x * cos(theta.toRad()) - v.y * sin(theta.toRad()),
        v.x * sin(theta.toRad()) + v.y * cos(theta.toRad()),
    )
    operator fun times(r: Rotation2d) = Rotation2d(theta + r.theta)
    val inverse get() = Rotation2d(-theta)
}
typealias Degrees = Double
typealias Rad = Double
fun Degrees.toRad() = 2 * PI / 360 * this
fun Rad.toDegrees() = 360 / (2 * PI) * this

// TODO: uses twists instead
/**
 * A [position] and [heading] in a 2D plane.
 */
data class Pose2d(val position: Vector2d, val heading: Rotation2d) {
    constructor(x: Double, y: Double, theta: Double) : this(Vector2d(x, y), Rotation2d(theta))
    constructor(position: Vector2d, theta: Double) : this(position, Rotation2d(theta))
    constructor(x: Double, y: Double, heading: Rotation2d) : this(Vector2d(x, y), heading)
    operator fun plus(p: Pose2d) = Pose2d(position + p.position, p.heading * heading)
    operator fun minus(p: Pose2d) = Pose2d(position - p.position, p.heading.inverse * heading)
    operator fun times(k: Double) = Pose2d(position * k, Rotation2d(heading.theta * k))
}