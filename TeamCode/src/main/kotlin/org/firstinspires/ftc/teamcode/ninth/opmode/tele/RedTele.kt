package org.firstinspires.ftc.teamcode.ninth.opmode.tele

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Color.*

@TeleOp(name="RED Tele")
class RedTele : Tele() {
    override val goodColors = setOf(YELLOW, RED)
    override val badColors = setOf(NONE, BLUE)
}