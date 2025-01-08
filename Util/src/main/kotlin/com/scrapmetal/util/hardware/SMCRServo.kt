package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import kotlin.math.abs

/**
 * CRServo wrapper with pwm ranges, cached writes, and opinionated configuration
 */
class SMCRServo(
    hardwareMap: HardwareMap,
    name: String,
    dir: DcMotorSimple.Direction,
    pwm: ModelPWM,
    private val eps: Double = 0.005,
    usFrame: Double = 4000.0,
) {
    private val crServo = hardwareMap.crservo.get(name) as CRServoImplEx
    private var _effort = 0.0

    var effort
        get() = _effort
        set(value) = if (abs(value - _effort) > eps) {
            _effort = value
        } else Unit

    init {
        crServo.direction = dir
        crServo.pwmRange = PwmControl.PwmRange(pwm.min, pwm.max, usFrame)
    }

    fun write() { crServo.power = effort }

    enum class ModelPWM(val min: Double, val max: Double) {
        AXON(500.0, 2500.0),
        GOBILDA_TORQUE(900.0, 2100.0), GOBILDA_SPEED(1000.0, 2000.0), GOBILDA_SUPER(1000.0, 2000.0),
    }
}