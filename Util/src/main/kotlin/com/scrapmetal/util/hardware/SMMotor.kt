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
    dir: Direction,
    zpb: ZeroPowerBehavior,
    val eps: Double = 0.005,
    currentThresh: Double = 9.0,
) {
    private val motor = hardwareMap.dcMotor.get(name) as DcMotorEx
    private var _effort = 0.0

    var effort
        get() = _effort
        set(value) = if (abs(value - _effort) > eps) {
            _effort = value
        } else Unit

    /**
     * Perform hardware write
     */
    fun write() { motor.power = effort }

    val isOverCurrent
        get() = motor.isOverCurrent

    init {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = dir
        motor.zeroPowerBehavior = zpb
        motor.setCurrentAlert(currentThresh, CurrentUnit.AMPS)
    }
}