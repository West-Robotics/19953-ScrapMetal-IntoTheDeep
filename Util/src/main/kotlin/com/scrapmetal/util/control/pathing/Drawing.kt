package com.scrapmetal.util.control.pathing

import com.acmerobotics.dashboard.canvas.Canvas
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Vector2d

const val ROBOT_COLOR = "#00ff00"
const val PATH_COLOR = "#0000ff"
const val ROBOT_RADIUS = 7.5
const val POINT_COUNT = 256
const val HEADING_COUNT = 16
const val TRAIL_COUNT = 256
const val TRAIL_RADIUS = 0.5
// TODO: does this need a flush between opmodes?
// TODO: use a better data structure
val trail = ArrayDeque<Vector2d>(512)

fun Canvas.drawRobot(pose: Pose2d): Canvas {
    val headingTip = pose.position + pose.heading*Vector2d(ROBOT_RADIUS, 0.0)
    this.setStroke(ROBOT_COLOR)
        .strokeCircle(pose.position.x, pose.position.y, ROBOT_RADIUS)
        .strokeLine(pose.position.x, pose.position.y, headingTip.x, headingTip.y)
    return this
}

fun Canvas.drawPath(spline: Spline): Canvas {
    val points = Array(POINT_COUNT) { spline(it/POINT_COUNT.toDouble()) }
    val xPoints = DoubleArray(POINT_COUNT) { points[it].x }
    val yPoints = DoubleArray(POINT_COUNT) { points[it].y }
    this.setStroke(PATH_COLOR)
        .strokePolyline(xPoints, yPoints)
    return this
}

/**
 * Draws the path along with interspersed heading markers
 */
fun Canvas.drawSubMovement(subMovement: SubMovement): Canvas {
    val points = Array(POINT_COUNT) { subMovement.spline(it / POINT_COUNT.toDouble()) }
    val xPoints = DoubleArray(POINT_COUNT) { points[it].x }
    val yPoints = DoubleArray(POINT_COUNT) { points[it].y }
    val headingTipPoints = Array(HEADING_COUNT) {
        Pair(
            points[it * POINT_COUNT / HEADING_COUNT],
            points[it * POINT_COUNT / HEADING_COUNT] +
                subMovement.heading(it / HEADING_COUNT.toDouble()) * Vector2d(ROBOT_RADIUS / 4, 0.0)
        )
    }
    this.setStroke(PATH_COLOR)
        .strokePolyline(xPoints, yPoints)
    headingTipPoints.forEach { this.strokeLine(it.first.x, it.first.y, it.second.x, it.second.y) }
    return this
}

fun Canvas.drawTrail(position: Vector2d): Canvas {
    if (trail.size == TRAIL_COUNT) trail.removeFirst()
    trail.add(position)
    this.setFill(ROBOT_COLOR)
    trail.forEach { this.fillCircle(it.x, it.y, TRAIL_RADIUS) }
    return this
}