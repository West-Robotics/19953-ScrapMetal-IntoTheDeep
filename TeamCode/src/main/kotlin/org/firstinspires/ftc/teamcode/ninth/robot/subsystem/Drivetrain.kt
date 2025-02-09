package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.constrainEffort
import com.scrapmetal.util.control.pControl
import com.scrapmetal.util.control.toDegrees
import com.scrapmetal.util.hardware.GoBildaPinpointDriver
import com.scrapmetal.util.hardware.SMMotor
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.newSingleThreadContext
import org.firstinspires.ftc.robotcore.external.Telemetry
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
    val pinpoint: GoBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint")
    // TODO: restore private

    private val kPTrans = 0.6
    private val kDTrans = 0.07
//    private val kPTrans = 0.0
//    private val kDTrans = 0.0
    private val kPRot = 0.09
    private val kDRot = 0.006
//    private val kPRot = 0.0
//    private val kDRot = 0.0

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

    fun setWheels(frontLeft: Double, backLeft: Double, backRight: Double, frontRight: Double) {
        this.frontLeft.effort = frontLeft
        this.backLeft.effort = backLeft
        this.backRight.effort = backRight
        this.frontRight.effort = frontRight
    }

    fun setPose(x: Double, y: Double, heading: Double) {
        pinpoint.position = Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading)
    }

    // TODO: thread?
    fun getPoseAndVelo(): Pair<Pose2d, Pose2d> = getPoseAndVelo(null)

    fun getPoseAndVelo(debug: Telemetry?): Pair<Pose2d, Pose2d> = Pair(
        pinpoint.position.let {
            Pose2d(
                it.getX(DistanceUnit.INCH),
                it.getY(DistanceUnit.INCH),
                it.getHeading(AngleUnit.DEGREES),
            )
        },
        pinpoint.velocity.let {
            Pose2d(
                it.getX(DistanceUnit.INCH),
                it.getY(DistanceUnit.INCH),
                pinpoint.headingVelocity.toDegrees(),
            )
        },
    ).also {
        if (debug != null) {
            debug.addData("x", it.first.position.x)
            debug.addData("y", it.first.position.y)
            debug.addData("heading", it.first.heading.theta)
            debug.addData("x velo", it.second.position.x)
            debug.addData("y velo", it.second.position.y)
            debug.addData("heading velo", it.second.heading.theta)
        }
    }

    // i don't like having control logic in the subsystem class but this would probably be the easiest
    //   with our current architecture
    // WARNING: maxTransEffort is for each individual axis, not the total magnitude
    fun getPDEffort(
        reference: Pose2d,
        maxTransEffort: Double = 1.0,
        maxRotEffort: Double = 1.0,
    ): Pose2d = getPDEffort(reference, maxTransEffort, maxRotEffort, debug = null)

    fun getPDEffort(
        reference: Pose2d,
        maxTransEffort: Double = 1.0,
        maxRotEffort: Double = 1.0,
        debug: Telemetry?,
    ): Pose2d {
        val (pose, velo) = getPoseAndVelo()
        val fieldFrameEffort = Pose2d(
            (pControl(kPTrans, reference.position.x, pose.position.x) +
                    pControl(kDTrans, 0.0, velo.position.x)).coerceIn(-maxTransEffort..maxTransEffort),
            (pControl(kPTrans, reference.position.y, pose.position.y) +
                    pControl(kDTrans, 0.0, velo.position.y)).coerceIn(-maxTransEffort..maxTransEffort),
            (pControl(kPRot, reference.heading.theta, pose.heading.theta, wraparound = true) +
                    pControl(kDRot, 0.0, velo.heading.theta)).coerceIn(-maxRotEffort..maxRotEffort),
        )
        val robotFrameEffort = Pose2d(pose.heading.inverse() * fieldFrameEffort.position, fieldFrameEffort.heading)

        if (debug != null) {
            debug.addData("x error", reference.position.x - pose.position.x)
            debug.addData("y error", reference.position.y - pose.position.y)
            debug.addData("heading error", reference.heading.theta - pose.heading.theta)
            debug.addData("x effort", robotFrameEffort.position.x)
            debug.addData("y effort", robotFrameEffort.position.y)
            debug.addData("heading effort", robotFrameEffort.heading.theta * 100)
        }
        return robotFrameEffort
    }

    @OptIn(ExperimentalCoroutinesApi::class, DelicateCoroutinesApi::class)
    fun beginPinpoint(scope: CoroutineScope) = scope.launch(newSingleThreadContext("pinpoint")) {
        while (isActive) {
            pinpoint.update()
        }
    }

    fun write() {
        frontLeft.write()
        backLeft.write()
        backRight.write()
        frontRight.write()
    }
}