package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import kotlin.math.abs

/**
 * Servo wrapper with pwm ranges, cached writes, and opinionated configuration
 */
class SMServo(
    hardwareMap: HardwareMap,
    name: String,
    initPos: Double,
    dir: Servo.Direction,
    pwm: ModelPWM,
    private val eps: Double = 0.005,
    usFrame: Double = 4000.0,
) {
    private val servo = hardwareMap.servo.get(name) as ServoImplEx
    private var _position = initPos.coerceIn(0.0, 1.0)

    var position
        get() = _position
        set(value) = if (abs(value - _position) > eps) {
            _position = value
        } else Unit

    init {
        servo.direction = dir
        servo.pwmRange = PwmControl.PwmRange(pwm.min, pwm.max, usFrame)
    }

    fun write() { servo.position = position }

    /**
     * Axons are limited from 510-2490 to prevent accidental wraparound (may be adjusted in the future)
     */
    enum class ModelPWM(val min: Double, val max: Double) {
        AXON(510.0, 2490.0),
        GENERIC(500.0, 2500.0),
    }
}