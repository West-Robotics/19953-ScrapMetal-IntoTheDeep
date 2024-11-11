package ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import ninth.robot.subsystem.DriveSubsystem
import ninth.robot.subsystem.IntakeSubsystem
import ninth.robot.subsystem.LiftSubsystem

@TeleOp(name = "NinthTele")
class Teleop: LinearOpMode() {
    override fun runOpMode() {
        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()

        val driveSubsystem = DriveSubsystem(hardwareMap)
        val liftSubsystem = LiftSubsystem(hardwareMap)
        val intakeSubsystem = IntakeSubsystem(hardwareMap)

        waitForStart()
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)
            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            // drive
            val throttlePower = (currentGamepad1.left_stick_y).toDouble()
            val strafePower = (currentGamepad1.left_stick_x).toDouble()
            val turnPower = (currentGamepad1.right_stick_x).toDouble()
            driveSubsystem.setVelocity(throttlePower, strafePower, turnPower)

            // virtical extension
            val desiredPos = when {
                (currentGamepad2.a) -> 60.0
                (currentGamepad2.b) -> 90.0
                else -> 0.0
            }

            val liftHeight = liftSubsystem.getHeight()
            val liftSpeed = 0.7

            if ((currentGamepad2.a) or (currentGamepad2.b)) {
                liftSubsystem.extensionSpeed(liftSubsystem.runToPreset(desiredPos, liftHeight, liftSpeed))
            } else {
                liftSubsystem.extensionSpeed((currentGamepad2.right_stick_y).toDouble())
            }

            // horizontal extension & intake
        }
    }
}