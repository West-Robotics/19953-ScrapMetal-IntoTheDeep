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
 * @param thresh minimum change in commanded power to necessitate a hardware write
 */
class SMCrServo(
    hardwareMap: HardwareMap,
    name: String,
    pwm: ModelPWM,
    private var thresh: Double = 0.005
) {
    enum class ModelPWM(val min: Double, val max: Double) {
        CR_AXON_MAX(510.0, 2490.0), CR_AXON_MINI(510.0, 2490.0), CR_AXON_MICRO(510.0, 2490.0),
        CR_GOBILDA_TORQUE(900.0, 2100.0), CR_GOBILDA_SPEED(1000.0, 2000.0), CR_GOBILDA_SUPER(1000.0, 2000.0),
    }

    private val crServo = hardwareMap.get(CRServoImplEx::class.java, name)
    private var lastEffort = 0.0

    init {
        crServo.pwmRange = PwmControl.PwmRange(pwm.min, pwm.max)
    }

    fun position(direction: DcMotorSimple.Direction) {
        crServo.direction = direction
    }

    fun thresh(thresh: Double) {
        this.thresh = thresh
    }

    fun effort(effort: Double) {
        if (abs(effort - lastEffort) > thresh) {
            crServo.power = effort
            lastEffort = effort
        }
    }

    fun getCommandedPower() = crServo.power

}