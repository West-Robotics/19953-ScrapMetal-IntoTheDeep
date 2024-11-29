package com.scrapmetal.util.control.pathing

import com.scrapmetal.util.control.Rotation2d
import kotlin.math.atan2

interface HeadingInterpolation {
    operator fun invoke(t: Double): Rotation2d
}

data class Constant(private val heading: Rotation2d) : HeadingInterpolation {
    constructor(theta: Double) : this(Rotation2d(theta))
    override fun invoke(t: Double) = heading
}

data class Tangent(private val spline: Spline) : HeadingInterpolation {
    override fun invoke(t: Double) = spline.tangentAt(t).let {
        Rotation2d(atan2(it.y, it.x))
    }
}

data class Linear(val start: Rotation2d, private val end: Rotation2d) : HeadingInterpolation {
    constructor(theta0: Double, theta1: Double) : this(Rotation2d(theta0), Rotation2d(theta1))
    override fun invoke(t: Double) = Rotation2d((end.theta - start.theta) * t) * start
}