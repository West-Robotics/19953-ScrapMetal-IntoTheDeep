package org.firstinspires.ftc.teamcode.ninth.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@TeleOp(name = "SubsystemTests")
class SubsystemTests : LinearOpMode() {
    override fun runOpMode() {
        val previousGamepad1 = Gamepad()
        val currentGamepad1 = Gamepad()

        // val drivetrain = Drivetrain(hardwareMap)
        val sampler = Sampler(hardwareMap)
        // val lift = Lift(hardwareMap)
        var desiredPos = 0.0
        var manual = false
        var extensionAmount = 0.43
//        var intakeToggle = false
//        var extensionToggle = false
//        var pitch = 0.1
//        var subPitch = 0.11
//        var roll = 0.1
//        var pinch = 0.0
//        var pto_position = 0.5

        // val dashboard = FtcDashboard.getInstance()
        // telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        waitForStart()
        sampler.state = Sampler.State.DEBUG
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            currentGamepad1.copy(gamepad1)

//            sampler.stow()
//            sampler.write()

//            lift.read()
//            if (gamepad1.a && !previousGamepad1.a) {
//                lift.setPreset(Lift.Preset.BOTTOM)
//            }
//            if (gamepad1.b && !previousGamepad1.b) {
//                lift.setPreset(Lift.Preset.SAMP_HIGH)
//            }
//            lift.updateProfiled(lift.getHeight(), telemetry)
//            lift.write()
//            telemetry.addData("height", lift.getHeight())
//            if (gamepad1.a && !previousGamepad1.a) {
//                pto_position = 0.75
//            }
//            if (gamepad1.b && !previousGamepad1.b) {
//                pto_position = 0.89
//            }
//            lift.pto.position = pto_position
//            lift.pto.write()

//            if (gamepad1.a && previousGamepad1.a) {
//                desiredPos = 0.0
//            }
//            if (gamepad1.b && previousGamepad1.b) {
//                desiredPos = 25.75 - 9
//            }
//            if (gamepad1.y && previousGamepad1.y) {
//                desiredPos = 43.0 - 9
//            }
//
//            if (gamepad1.start && !previousGamepad1.start) {
//                manual = !manual
//            }

            // drive
//            drivetrain.setVelocity(
//                -gamepad1.left_stick_y.toDouble(),
//                -gamepad1.left_stick_x.toDouble(),
//                -gamepad1.right_stick_x.toDouble(),
//            )
//            drivetrain.write()

            // lift
//            val liftHeight = lift.getHeight()

//            if (!manual) {
//                lift.runToPos(desiredPos, liftHeight)
//            } else {
//              lift.setEffort(gamepad1.left_trigger - gamepad1.right_trigger.toDouble() + 0.20)
//            }
//
//            lift.write()

            // sampler

//            if (currentGamepad1.a && !previousGamepad1.a) {
//                if (extensionToggle) {
//                    sampler.extend()
//                } else {
//                    sampler.retract()
//                }
//                extensionToggle = !extensionToggle
//            }
//
//            if (currentGamepad1.b && !previousGamepad1.b) {
//                if (intakeToggle) {
//                    sampler.grab()
//                } else {
//                    sampler.stow()
//                }
//                intakeToggle = !intakeToggle
//            }
//
//            if (currentGamepad1.x && !previousGamepad1.x) {
//                sampler.score()
//            }
//            if (currentGamepad1.y && !previousGamepad1.y) {
//                extensionAmount = 0.0
//            }
//            if (currentGamepad1.y && previousGamepad1.y) {
//                extensionAmount = 0.68
//            }

            // if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
            //     sampler.extend()
            // }
            // if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
            //     sampler.grab_sample()
            // }

            // if (gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            //     sampler.grab_sample_right_side()
            // }
            // if (gamepad1.dpad_left && !previousGamepad1.dpad_left) {
            //     sampler.grab_sample_left_side()
            // }

//            if (gamepad1.triangle && !previousGamepad1.triangle) {
//                roll += 0.01
//            }
//            if (gamepad1.cross && !previousGamepad1.cross) {
//                roll -= 0.01
//            }
//
            // if (gamepad1.circle && !previousGamepad1.circle) {
            //     sampler.claw.position = 0.27
            // }
            // if (gamepad1.square && !previousGamepad1.square) {
            //     sampler.claw.position = 0.0
            // }

            // if (gamepad1.b) {
            //     sampler.setState(Sampler.State.STOW)
            // }
            // if (gamepad1.y) {
            //     sampler.setState(Sampler.State.EXTEND)
            // }
            // if (gamepad1.x) {
            //     sampler.setState(Sampler.State.GRAB_SAMP)
            // }
            // if (gamepad1.dpad_left) {
            //     sampler.setState(Sampler.State.PICKED)
            // }
            // if (gamepad1.dpad_down) {
            //     sampler.setState(Sampler.State.HOLD_SAMP)
            // }
            // if (gamepad1.right_bumper) {
            //     sampler.setState(Sampler.State.PREP_SCORE_SAMP)
            // }
            // if (gamepad1.left_bumper) {
            //     sampler.setState(Sampler.State.SCORE_SAMP)
            // }

            sampler.write()
            sampler.updateProfiled()

//            telemetry.addData("sub pitch", subPitch)
//            telemetry.addData("pitch", pitch)
//            telemetry.addData("roll", roll)
//            telemetry.addData("pinch", pinch)


//            telemetry.addLine("Raise lift - hold left trigger")
//            telemetry.addLine("Lower lift - hold right trigger")
//
//            telemetry.addLine("Extend sampler - press a")
//            telemetry.addLine("Retract sampler - press a")
//            telemetry.addLine("Intake sample - press b")
//            telemetry.addLine("Stow intake - press b")
//            telemetry.addLine("Score - hold x")
//            telemetry.addLine("           ")
//            telemetry.addData("Lift height", liftHeight)
//            telemetry.addData("Control Effort", lift.getEffort())
//            telemetry.addData("Desired position", desiredPos)
//            telemetry.addData("Scoring pos", score)
//            telemetry.addData("grabbing spec wrist", grab_wrist)
//            telemetry.addData("grabbing spec pivot", grab_spec)
//            telemetry.addData("pto position", pto_position)

            telemetry.update()
        }
    }
}