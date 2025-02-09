package org.firstinspires.ftc.teamcode.ninth

fun inchesToTicks(inches: Double, cpr:Double, spoolCircumference: Double) = (inches * cpr/spoolCircumference).toInt()

fun ticksToInches(ticks: Int, spoolCircumference: Double, cpr: Double) = ticks * spoolCircumference / cpr

fun controlEffort(preset: Double, currentHeight: Double, kp:Double, feedforward:Double) = kp*(preset-currentHeight) + if (currentHeight > 0.1) feedforward else 0.0

val NOM_VOLT = 13.0
val LENGTH = 15.5
val WIDTH = 14.2