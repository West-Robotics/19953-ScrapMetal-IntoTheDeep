package org.firstinspires.ftc.teamcode.ninth.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.pathing.Constant
import com.scrapmetal.util.control.pathing.Follower
import com.scrapmetal.util.control.pathing.LinePoint
import com.scrapmetal.util.control.pathing.Linear
import com.scrapmetal.util.control.pathing.SplinePoint
import com.scrapmetal.util.control.pathing.drawMovement
import com.scrapmetal.util.control.pathing.drawSubMovement
import com.scrapmetal.util.control.pathing.lineTo
import com.scrapmetal.util.control.pathing.splineTo
import com.scrapmetal.util.control.pathing.withSpeed
import com.scrapmetal.util.control.pathing.withHeading
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.ninth.NOM_VOLT
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.opmode.test.FollowerTest.State.*

@Autonomous(name="Follower Test")
class FollowerTest : LinearOpMode() {
    enum class State {
        LINES,
        SPLINE_C,
        SPLINE_TH,
    }

    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))

        val follower = Follower(kN=0.5, kP=0.5, kD=0.05, kTheta=0.08, kOmega=0.005, endDistance=12.0)
        val p0 = Vector2d(0.0, 0.0)
        val p1 = Vector2d(24.0, 0.0)
        val p2 = Vector2d(24.0, -12.0)
        val p3 = Vector2d(48.0, 0.0)

        val lines = LinePoint(p0) lineTo
                LinePoint(p1) withHeading Constant(45.0) withSpeed 0.4 lineTo
                LinePoint(p2) withSpeed 0.2 lineTo
                LinePoint(p3) withHeading Linear(-90.0, +90.0)
        val splineConstantHeading = SplinePoint(p3, 80.0, 135.0) splineTo SplinePoint(p0, 80.0, 135.0) withHeading Constant(135.0)
        val splineTangentHeading = SplinePoint(p0, 80.0, 45.0) splineTo SplinePoint(p3, 80.0, 45.0)

        val fsm = StateMachineBuilder()
            .state(LINES)
            .onEnter { follower.follow(lines) }
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.2) }
            .state(SPLINE_C)
            .onEnter { follower.follow(splineConstantHeading) }
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.2) }
            .state(SPLINE_TH)
            .onEnter { follower.follow(splineTangentHeading) }
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.2) }
            .build()

        val dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        val packet = TelemetryPacket()
        packet.fieldOverlay()
            .drawMovement(splineTangentHeading)
        dashboard.sendTelemetryPacket(packet)
        waitForStart()
        drivetrain.setPose(Pose2d(p0, 0.0))
        fsm.start()
        while (opModeIsActive()) {
            drivetrain.read()
            fsm.update()
            val (pose, velo) = drivetrain.getPoseAndVelo()
            drivetrain.setEffort(follower.update(pose, velo))
            drivetrain.write()
            telemetry.update()
        }
    }
}
