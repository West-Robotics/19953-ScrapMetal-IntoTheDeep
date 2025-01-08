package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs

/**
 * Motor wrapper with cached writes and opinionated configuration
 */
class SMMotor(
    hardwareMap: HardwareMap,
    name: String,
    dir: Direction,
    zpb: ZeroPowerBehavior,
    private val eps: Double = 0.005,
    currentThresh: Double = 8.0,
) {
    private val motor = hardwareMap.dcMotor.get(name) as DcMotorEx
    private var _effort = 0.0

    var effort
        get() = _effort
        set(value) = if (abs(value - _effort) > eps) {
            _effort = value
        } else Unit

    val isOverCurrent
        get() = motor.isOverCurrent

    val current
        get() = motor.getCurrent(CurrentUnit.AMPS)

    init {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = dir
        motor.zeroPowerBehavior = zpb
        motor.setCurrentAlert(currentThresh, CurrentUnit.AMPS)
    }

    /**
     * Perform hardware write
     */
    fun write() { motor.power = effort }

}