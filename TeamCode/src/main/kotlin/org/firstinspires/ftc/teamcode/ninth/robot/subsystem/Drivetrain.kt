package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.hardware.GoBildaPinpointDriver
import com.scrapmetal.util.hardware.SMMotor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import kotlin.math.abs
import kotlin.math.max

class Drivetrain(hardwareMap: HardwareMap, val voltageMultiplier: Double = 1.0) {
    private val frontLeft = SMMotor(hardwareMap, "frontLeft", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE)
    private val backLeft = SMMotor(hardwareMap, "backLeft", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE)
    private val backRight = SMMotor(hardwareMap, "backRight", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE)
    private val frontRight = SMMotor(hardwareMap, "frontRight", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE)
    val pinpoint = hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint")

    init {
        // 3.75 m, 6.5 + 1/16 in
        pinpoint.setOffsets(-3.75, -166.6875)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD)
        pinpoint.resetPosAndIMU()
    }

    fun setEffort(effort: Pose2d) {
        (effort * voltageMultiplier).let { effort ->
            val yModified: Double = effort.position.y * 1.0
            val denominator = max(abs(effort.position.x) + abs(effort.position.y) + abs(effort.heading.theta), 1.0)

            frontLeft.effort = (effort.position.x - yModified - effort.heading.theta) / denominator
            backLeft.effort = (effort.position.x + yModified - effort.heading.theta) / denominator
            backRight.effort = (effort.position.x - yModified + effort.heading.theta) / denominator
            frontRight.effort = (effort.position.x + yModified + effort.heading.theta) / denominator
        }
    }

    fun setEffort(x: Double, y: Double, turn: Double) = setEffort(Pose2d(x, y, turn))

    fun setPose(x: Double, y: Double, heading: Double) {
        pinpoint.position = Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading)
    }

    fun getPose(): Pose2d {
        pinpoint.update()
        return pinpoint.position.let {
            Pose2d(
                it.getX(DistanceUnit.INCH),
                it.getY(DistanceUnit.INCH),
                it.getHeading(AngleUnit.DEGREES)
            )
        }
    }

    fun write() {
        frontLeft.write()
        backLeft.write()
        backRight.write()
        frontRight.write()
    }
}
