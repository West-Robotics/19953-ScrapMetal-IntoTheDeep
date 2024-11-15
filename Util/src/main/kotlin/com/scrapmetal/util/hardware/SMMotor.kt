package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs

/**
 * Motor wrapper with cached writes (with [eps]) and opinionated configuration
 */
class SMMotor(
    hardwareMap: HardwareMap,
    name: String,
    val eps: Double = 0.005,
    dir: Direction,
    zpb: ZeroPowerBehavior,
    currentThresh: Double = 9.0,
) {
    private val motor = hardwareMap.dcMotor.get(name) as DcMotorEx
    private var previousEffort = 0.0

    var effort
        get() = previousEffort
        set(value) = if (abs(effort - previousEffort) > eps) {
            motor.power = value
            previousEffort = value
        } else Unit

    val isOverCurrent
        get() = motor.isOverCurrent

    init {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = dir
        motor.zeroPowerBehavior = zpb
        motor.setCurrentAlert(currentThresh, CurrentUnit.AMPS)
    }
}