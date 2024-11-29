package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap

class SMQuadrature(
    hardwareMap: HardwareMap,
    name: String,
    val distPerTick: Double,
    val revsPerTick: Double,
    val dir: Direction,
) {
    private val motor = hardwareMap.dcMotor.get(name) as DcMotorEx
    private var tickOffset = 0

    val dist
        get() = ticks * distPerTick
    val revs
        get() = ticks * revsPerTick
    val ticks
        get() = when (dir) {
            Direction.FORWARD -> 1
            Direction.REVERSE -> -1
        } * (motor.currentPosition - tickOffset)

    val linearV
        get() = tickV * distPerTick
    // TODO: verify units
    val angularV
        get() = tickV * revsPerTick
    // TODO: add velocity overflow correction
    // TODO: apply filter/use different estimate
    // TODO: verify units
    val tickV
        get() = motor.velocity

    fun reset(distance: Double = 0.0) {
        tickOffset = motor.currentPosition - (distance/distPerTick).toInt()
    }
}