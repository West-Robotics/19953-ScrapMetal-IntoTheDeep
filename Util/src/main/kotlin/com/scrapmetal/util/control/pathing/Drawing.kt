package com.scrapmetal.util.control.pathing

import com.acmerobotics.dashboard.canvas.Canvas
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Vector2d

const val robotColor = "#00ff00"
const val pathColor = "#0000ff"
const val robotRadius = 7.5
const val pointCount = 256
const val headingCount = 16
const val trailCount = 512
const val trailRadius = 0.125
// TODO: does this need a flush between opmodes?
// TODO: use a better data structure
val trail = ArrayDeque<Vector2d>(512)

fun Canvas.drawRobot(pose: Pose2d): Canvas {
    val headingTip = pose.position + pose.heading*Vector2d(robotRadius, 0.0)
    this.setStroke(robotColor)
        .strokeCircle(pose.position.x, pose.position.y, robotRadius)
        .strokeLine(pose.position.x, pose.position.y, headingTip.x, headingTip.y)
    return this
}

fun Canvas.drawPath(spline: Spline): Canvas {
    val points = Array(pointCount) { spline(it/pointCount.toDouble()) }
    val xPoints = DoubleArray(pointCount) { points[it].x }
    val yPoints = DoubleArray(pointCount) { points[it].y }
    this.setStroke(pathColor)
        .strokePolyline(xPoints, yPoints)
    return this
}

fun Canvas.drawPathWithHeading(spline: Spline, heading: HeadingInterpolation): Canvas {
    val points = Array(pointCount) { spline(it/pointCount.toDouble()) }
    val xPoints = DoubleArray(pointCount) { points[it].x }
    val yPoints = DoubleArray(pointCount) { points[it].y }
    val headingTipPoints = Array(headingCount) {
        Pair(
            points[it*pointCount/headingCount],
            points[it*pointCount/headingCount] +
                heading(it/headingCount.toDouble())*Vector2d(robotRadius/4, 0.0)
        )
    }
    this.setStroke(pathColor)
        .strokePolyline(xPoints, yPoints)
    headingTipPoints.forEach { this.strokeLine(it.first.x, it.first.y, it.second.x, it.second.y) }
    return this
}

fun Canvas.drawTrail(position: Vector2d): Canvas {
    if (trail.size == trailCount) trail.removeFirst()
    trail.add(position)
    this.setStroke(robotColor)
    trail.forEach { this.fillCircle(it.x, it.y, trailRadius) }
    return this
}