package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime

class SMQuadrature(
    hardwareMap: HardwareMap,
    name: String,
    val distPerTick: Double,
    val revsPerTick: Double,
    val dir: Direction,
    val beta: Double = 1.0,
) {
    private val motor = hardwareMap.dcMotor.get(name) as DcMotorEx
    private val reversal = if (motor.direction == Direction.FORWARD) 1 else -1
    private var tickOffset = 0
    private val updateTimer = ElapsedTime()

    val dist
        get() = ticks * distPerTick
    val revs
        get() = ticks * revsPerTick
    var ticks = motor.currentPosition
        private set

    val linearV
        get() = tickV * distPerTick
    // TODO: verify units
    val angularV
        get() = tickV * revsPerTick
    // TODO: add velocity overflow correction
    // TODO: apply filter/use different estimate
    // TODO: verify units
    var tickV = 0.0
        private set

    fun update() {
        val newTicks = when (dir) {
            Direction.FORWARD -> 1
            Direction.REVERSE -> -1
        } * reversal * (motor.currentPosition - tickOffset)
        val dt = updateTimer.seconds()
        updateTimer.reset()
        val veloEst = (newTicks - ticks) / dt
        tickV += beta * (veloEst - tickV)
        ticks = newTicks
    }

    fun reset(distance: Double = 0.0) {
        tickOffset = motor.currentPosition - (distance/distPerTick).toInt()
    }

    fun trueReset() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}