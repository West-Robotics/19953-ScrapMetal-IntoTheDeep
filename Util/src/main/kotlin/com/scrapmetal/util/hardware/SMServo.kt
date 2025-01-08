package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import kotlin.math.abs

/**
 * Servo wrapper with cached writes, built in servo pwm ranges, and fewer functions
 */

// TODO: add direction and initial pos as part of constructor so they are explicitly stated

class SMServo(
    hardwareMap: HardwareMap,
    name: String,
    dir: Servo.Direction,
    pwm: ModelPWM,
    val eps: Double = 0.005,
    usFrame: Double = 5000.0,
) {
    private val servo = hardwareMap.get(ServoImplEx::class.java, name)
    private var _position = 0.0

    init {
        servo.pwmRange = PwmControl.PwmRange(pwm.min, pwm.max, usFrame)
        servo.direction = dir
    }

    fun direction(direction: Servo.Direction) {
        servo.direction = direction
    }

    var position
        get() = _position
        set(value) = if (abs(value - _position) > eps) {
            _position = value
        } else Unit

    fun write() { servo.position = position }

    /**
     * Axons are limited from 510-2490 to prevent accidental wraparound (may be adjusted) in the future
     */

    enum class ModelPWM(val min: Double, val max: Double) {
        AXON(510.0, 2490.0),
        GOBILDA_TORQUE(500.0, 2500.0), GOBILDA_SPEED(500.0, 2500.0), GOBILDA_SUPER(500.0, 2500.0),
        GENERIC(500.0, 2500.0),
    }
}