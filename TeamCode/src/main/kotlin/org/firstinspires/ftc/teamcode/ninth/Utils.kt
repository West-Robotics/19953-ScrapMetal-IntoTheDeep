package org.firstinspires.ftc.teamcode.ninth

fun inchesToTicks(inches: Double, cpr:Double, spoolCircumference: Double) = (inches * cpr/spoolCircumference).toInt()

fun ticksToInches(ticks: Int, spoolCircumference: Double, cpr: Double) = ticks * spoolCircumference / cpr

fun controlEffort(preset: Double, currentHeight: Double, kp:Double, feedforward:Double) = kp*(preset-currentHeight) + feedforward