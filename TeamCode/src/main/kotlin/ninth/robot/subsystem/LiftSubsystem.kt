package ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import kotlin.math.PI

class LiftSubsystem(hardwareMap: HardwareMap) {
    // remember to plug the encoder into the liftOne port!
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

    fun resetEncoder() {
        liftOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    }

    val driveCPR = 8192
    val wheelCircumference: Double = 10.4 * PI

    fun centimetersToTicks(cm: Double): Int {
        return (cm * driveCPR/wheelCircumference).toInt()
    }

    fun ticksToCentimeters(ticks: Int): Double {
        return (ticks * wheelCircumference / driveCPR)
    }

    fun extensionSpeed(speed: Double) {
        liftOne.setPower(speed)
        liftTwo.setPower(speed)
    }

    fun getHeight(): Double {
        val liftDisplacement = ticksToCentimeters(liftOne.getCurrentPosition())
        return liftDisplacement
    }

    fun runToPreset(preset: Double, currentHeight: Double, speed: Double): Double {
        val speedForLoop = when {
            currentHeight < preset -> speed
            currentHeight > preset -> -speed
            else -> 0.0
        }
        return speedForLoop
    }
}