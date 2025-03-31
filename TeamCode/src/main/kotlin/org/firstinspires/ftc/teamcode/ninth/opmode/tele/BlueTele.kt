package org.firstinspires.ftc.teamcode.ninth.opmode.tele

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Color.*

@TeleOp(name="BLUE Tele")
class BlueTele : Tele() {
    override val goodColors = setOf(YELLOW, BLUE)
    override val badColors = setOf(NONE, RED)
}
