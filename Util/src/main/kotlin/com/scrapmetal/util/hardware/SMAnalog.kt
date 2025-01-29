package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

class SMAnalog (
    hardwareMap: HardwareMap,
    name: String,
) {
    private val analogInput = hardwareMap.analogInput.get(name)
    private var lastPosition = analogInput.voltage / 3.3 * 360
    private val readTimer = ElapsedTime()
    val readEvery = ElapsedTime()
    var currentSpeed = 0.0

    init {
        readEvery.reset()
    }

    private fun updateSpeed() {
        val currentPosition = analogInput.voltage / 3.3 * 360
        currentSpeed = abs(currentPosition - lastPosition) / readTimer.seconds()
        readTimer.reset()
        lastPosition = currentPosition
        readEvery.reset()
    }

    val speed
        get() =
        if (readEvery.milliseconds() > 15) {
            updateSpeed()
            currentSpeed
        } else currentSpeed

    val getVoltage
        get() = analogInput.voltage
}