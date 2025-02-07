package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.pControl
import com.scrapmetal.util.hardware.GoBildaPinpointDriver
import com.scrapmetal.util.hardware.SMMotor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import kotlin.math.abs
import kotlin.math.max

class Drivetrain(hardwareMap: HardwareMap, private val voltageMultiplier: Double = 1.0) {
    private val frontLeft = SMMotor(hardwareMap, "frontLeft", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE)
    private val backLeft = SMMotor(hardwareMap, "backLeft", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE)
    private val backRight = SMMotor(hardwareMap, "backRight", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE)
    private val frontRight = SMMotor(hardwareMap, "frontRight", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE)
    private val pinpoint: GoBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint")

    private val kPTrans = 0.1
    private val kDTrans = 0.0
    private val kPRot = 0.1
    private val kDRot = 0.0

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

    // TODO: thread?
    fun getPoseAndVelo(): Pair<Pose2d, Pose2d> {
        return Pair(
            pinpoint.position.let {
                Pose2d(
                    it.getX(DistanceUnit.INCH),
                    it.getY(DistanceUnit.INCH),
                    it.getHeading(AngleUnit.DEGREES)
                )
            },
            pinpoint.velocity.let {
                Pose2d(
                    it.getX(DistanceUnit.INCH),
                    it.getY(DistanceUnit.INCH),
                    it.getHeading(AngleUnit.DEGREES)
                )
            },
        )
    }

    // i don't like having control logic in the subsystem class but this would probably be the easiest
    //   with our current architecture
    // WARNING: maxTransEffort is for each individual axis, not the total magnitude
    fun getPDEffort(
        reference: Pose2d,
        maxTransEffort: Double = 1.0,
        maxRotEffort: Double = 1.0,
    ): Pose2d {
        val (pose, velo) = getPoseAndVelo()
        val fieldFrameEffort = Pose2d(
            (pControl(kPTrans, reference.position.x, pose.position.x) +
                pControl(kDTrans, 0.0, velo.position.x)).coerceIn(-maxTransEffort..maxTransEffort),
            (pControl(kPTrans, reference.position.y, pose.position.y) +
                pControl(kDTrans, 0.0, velo.position.y)).coerceIn(-maxTransEffort..maxTransEffort),
            (pControl(kPRot, reference.heading.theta, pose.heading.theta, wraparound = true) +
                pControl(kDRot, 0.0, velo.heading.theta, wraparound = true)).coerceIn(-maxRotEffort..maxRotEffort),
        )
        return Pose2d(pose.heading.inverse() * fieldFrameEffort.position, fieldFrameEffort.heading)
    }

    fun read() = pinpoint.update()

    fun write() {
        frontLeft.write()
        backLeft.write()
        backRight.write()
        frontRight.write()
    }
}