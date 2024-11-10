package ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap

class LiftSubsystem {
    class DriveSubsystem {
        private val liftOne = hardwareMap.get(DcMotor::class.java, "liftOne")
        private val liftTwo = hardwareMap.get(DcMotor::class.java, "liftTwo")

        init {
            liftOne.setDirection(DcMotorSimple.Direction.FORWARD)
            liftTwo.setDirection(DcMotorSimple.Direction.REVERSE)

            liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

            liftOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            liftTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        }
    }
}