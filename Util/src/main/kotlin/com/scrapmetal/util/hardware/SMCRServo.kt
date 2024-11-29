package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import kotlin.math.abs

/**
 * CRServo wrapper with cached writes, built in servo pwm ranges, and less functions
 *
 * @param pwm servo model, used to determine pwm range
 * @param currentThresh minimum change in commanded power to necessitate a hardware write
 */
class SMCRServo(
    hardwareMap: HardwareMap,
    name: String,
    pwm: ModelPWM,
    dir: DcMotorSimple.Direction,
    val eps: Double = 0.005,
    private var currentThresh: Double = 0.005
) {
    enum class ModelPWM(val min: Double, val max: Double) {
        CR_AXON_MAX(510.0, 2490.0), CR_AXON_MINI(510.0, 2490.0), CR_AXON_MICRO(510.0, 2490.0),
        CR_GOBILDA_TORQUE(900.0, 2100.0), CR_GOBILDA_SPEED(1000.0, 2000.0), CR_GOBILDA_SUPER(1000.0, 2000.0),
    }

    private val crServo = hardwareMap.get(name) as CRServoImplEx
    private var _effort = 0.0

    var effort
        get() = _effort
        set(value) = if (abs(value - _effort) > eps) {
            _effort = value
        } else Unit

    fun position(direction: DcMotorSimple.Direction) {
    }

    fun thresh(thresh: Double) {
        this.currentThresh = thresh
    }

    fun commandedPower() = crServo.power

    /**
     * Perform hardware write
     */
    fun write() { crServo.power = effort }

    init {
        crServo.pwmRange = PwmControl.PwmRange(pwm.min, pwm.max)
        crServo.direction = dir
    }
}