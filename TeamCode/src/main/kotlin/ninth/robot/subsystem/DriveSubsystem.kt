package ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import kotlin.math.abs
import kotlin.math.max

class DriveSubsystem {
    private val frontLeft = hardwareMap.get(DcMotor::class.java, "frontLeft")
    private val backLeft = hardwareMap.get(DcMotor::class.java, "backLeft")
    private val backRight = hardwareMap.get(DcMotor::class.java, "backRight")
    private val frontRight = hardwareMap.get(DcMotor::class.java, "frontRight")


    init {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE)
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD)
        backRight.setDirection(DcMotorSimple.Direction.REVERSE)
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD)

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    fun drive(leftY:Double, leftX:Double, rightX:Double) {
        val leftXModified: Double = leftX * 1.1
        val denominator = max(((leftY) + abs(leftX) + abs(rightX)), 1.0)

        frontLeft.setPower((-leftY + leftX + rightX)/denominator)
        backLeft.setPower((-leftY - leftX + rightX)/denominator)
        backRight.setPower((-leftY + leftX - rightX)/denominator)
        frontRight.setPower((-leftY - leftX - rightX)/denominator)
    }
}
