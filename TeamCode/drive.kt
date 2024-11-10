
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class mechanumDrive {
    private val frontLeft = hardwareMap.get(DcMotor:: class.java, "frontLeft"
    private val backLeft = hardwareMap.get(DcMotor::class.java, "backLeft")
    private val frontRight = hardwareMap.get(DcMotor::class.java, "frontRight")
    private val backRight = hardwareMap.get(DcMotor::class.java, "backRight")

    init {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE)
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE)
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD)
        backRight.setDirection(DcMotorSimple.Direction.FORWARD)

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    fun resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
}