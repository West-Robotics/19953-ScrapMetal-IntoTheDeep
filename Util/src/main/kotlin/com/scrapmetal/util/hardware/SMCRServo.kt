package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import kotlin.math.abs

/**
 * CRServo wrapper with cached writes, built in servo pwm ranges, and fewer functions
 */

class SMCRServo(
    hardwareMap: HardwareMap,
    name: String,
    dir: DcMotorSimple.Direction,
    pwm: ModelPWM,
    val eps: Double = 0.005,
) {

    private val crServo = hardwareMap.crservo.get(name) as CRServoImplEx
    private var _effort = 0.0

    init {
        crServo.pwmRange = PwmControl.PwmRange(pwm.min, pwm.max)
        crServo.direction = dir
    }

    var effort
        get() = _effort
        set(value) = if (abs(value - _effort) > eps) {
            _effort = value
        } else Unit

    /**
     * Perform hardware write
     */
    fun write() { crServo.power = effort }

    enum class ModelPWM(val min: Double, val max: Double) {
        AXON(510.0, 2490.0),
        GOBILDA_TORQUE(900.0, 2100.0), GOBILDA_SPEED(1000.0, 2000.0), GOBILDA_SUPER(1000.0, 2000.0),
    }
}