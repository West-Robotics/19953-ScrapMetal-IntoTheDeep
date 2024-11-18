package org.firstinspires.ftc.teamcode.experimental

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update

class RobotState(hardwareMap: HardwareMap, val gamepad: Gamepad) {
    private val enc = hardwareMap.dcMotor.get("lift")
    private val _height = MutableStateFlow(0.0)
    val height = _height.asStateFlow()
    private val _forward = MutableStateFlow(0.0)
    val forward = _forward.asStateFlow()
    private val _turn = MutableStateFlow(0.0)
    val turn = _turn.asStateFlow()

    fun update() {
        _height.update { _ -> enc.currentPosition.toDouble() }
        _forward.update { _ -> -gamepad.left_stick_y.toDouble() }
        _turn.update { _ -> -gamepad.right_stick_x.toDouble() }
    }
}