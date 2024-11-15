package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import androidx.core.app.NotificationCompat.GroupAlertBehavior
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import kotlin.enums.enumEntries

class Sampler(hardwareMap: HardwareMap) {
    private val extensionOne = hardwareMap.get(Servo::class.java, "extOne")
    private val extensionTwo = hardwareMap.get(Servo::class.java, "extTwo")

    private val pivot = hardwareMap.get(Servo::class.java, "pivot")
    private val intake = hardwareMap.get(CRServo::class.java, "intake")

    init {
        pivot.setDirection(Servo.Direction.FORWARD)
        intake.setDirection(DcMotorSimple.Direction.FORWARD)
        extensionOne.setDirection(Servo.Direction.FORWARD)
        extensionTwo.setDirection(Servo.Direction.REVERSE)
    }

    enum class State(val grabber: Double, val pivot: Double, val linkage: Double) {
        EXTEND(0.0, 0.0, 0.0),
        GRAB(0.0, 0.0, 0.0),
        STOW(0.0, 0.0, 0.0),
        RETRACT(0.0, 0.0, 0.0),
        SCORE(0.0, 0.0, 0.0),
        RELEASE(0.0, 0.0, 0.0),
    }

    // have continuous power (but less than intake) going during stow
    fun setState(state: State) {
        intake.setPower(state.grabber)
        pivot.setPosition(state.pivot)
        extensionOne.setPosition(state.linkage)
        extensionTwo.setPosition(state.linkage)
    }

    fun extend() = setState(State.EXTEND)
    fun grab() = setState(State.GRAB)
    fun stow() = setState(State.STOW)
    fun retract() = setState(State.RETRACT)
    fun score() = setState(State.SCORE)
    fun release() = setState(State.RELEASE)

}