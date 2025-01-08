package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import kotlin.math.abs

/**
 * Servo wrapper with cached writes, built in servo pwm ranges, and fewer functions
 */

/**
 * @param pwm servo model, used to determine pwm range
 * @param pwm servo model, used to determine pwm range
 * @param usFrame framing rate in microseconds
 */

// TODO: add direction and initial pos as part of constructor so they are explicitly stated

class SMServo(
    hardwareMap: HardwareMap,
    name: String,
    pwm: ModelPWM,
    private var eps: Double = 0.002,
    usFrame: Double = 5000.0,
) {
    /**
     * PWM ranges for various servo models
     *
     * Axons are limited from 510-2490 to prevent accidental wraparound (may be adjusted) in the future
     */

    private val servo = hardwareMap.get(ServoImplEx::class.java, name)
    private var _position = 0.0

    init {
        servo.pwmRange = PwmControl.PwmRange(pwm.min, pwm.max, usFrame)
    }

    fun direction(direction: Servo.Direction) {
        servo.direction = direction
    }

    fun eps(thresh: Double) {
        this.eps = eps
    }

    /**
     * @param convertToPosition convert position argument to servo's 0-1 range
     */

    fun getCommandPosition() = servo.position

    var position
        get() = _position
        set(value) = if (abs(value - _position) > eps) {
            _position = value
        } else Unit

    fun write() { servo.position = position }

    enum class ModelPWM(val min: Double, val max: Double) {
        AXON(510.0, 2490.0),
        GOBILDA_TORQUE(500.0, 2500.0), GOBILDA_SPEED(500.0, 2500.0), GOBILDA_SUPER(500.0, 2500.0),
        GENERIC(500.0, 2500.0),
    }
}