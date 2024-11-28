package com.scrapmetal.util.control.pathing

import com.acmerobotics.dashboard.canvas.Canvas
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Vector2d

const val robotColor = "#00FF00"
const val pathColor = "#0000FF"
const val robotRadius = 7.5
const val pointCount = 256
const val trailCount = 512
const val trailRadius = 0.125
// TODO: does this need a flush between opmodes?
// TODO: use a better data structure
val trail = ArrayDeque<Vector2d>(512)

fun drawRobot(canvas: Canvas, pose: Pose2d) {
    val headingTip = pose.position + pose.heading*Vector2d(robotRadius, 0.0)
    canvas.setStroke(robotColor)
        .strokeCircle(pose.position.x, pose.position.y, robotRadius)
        .strokeLine(pose.position.x, pose.position.y, headingTip.x, headingTip.y)
}

// with heading?
fun drawPath(canvas: Canvas, spline: Spline) {
    val points = Array(pointCount) { spline(it/pointCount.toDouble()) }
    val xPoints = DoubleArray(pointCount) { points[it].x }
    val yPoints = DoubleArray(pointCount) { points[it].y }
    canvas.setStroke(pathColor)
        .strokePolyline(xPoints, yPoints)
}

fun drawTrail(canvas: Canvas, position: Vector2d) {
    if (trail.size == trailCount) trail.removeFirst()
    trail.add(position)
    canvas.setStroke(robotColor)
    trail.forEach { canvas.fillCircle(it.x, it.y, trailRadius) }
}