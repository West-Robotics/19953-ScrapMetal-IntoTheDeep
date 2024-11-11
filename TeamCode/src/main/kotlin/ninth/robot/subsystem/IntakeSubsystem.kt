package ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap

class IntakeSubsystem(hardwareMap: HardwareMap) {
    private val extensionOne = hardwareMap.get(Servo::class.java, "extOne")
    private val extensionTwo = hardwareMap.get(Servo::class.java, "extTwo")

    private val intakePivot = hardwareMap.get(Servo::class.java, "pivot")
    private val intakeWheel = hardwareMap.get(Servo::class.java, "intakeWheel")

    val retract = 0.0
    val intake = 0.0
    val output = 0.0

    init {
        extensionOne.setDirection(Servo.Direction.FORWARD)
        extensionTwo.setDirection(Servo.Direction.REVERSE)

        intakePivot.setPosition(retract)
    }
}