package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime

class SMAnalog (
    hardwareMap: HardwareMap,
    name: String,
) {
    private val analogInput = hardwareMap.analogInput.get(name)
    private var lastPosition = analogInput.voltage / 3.3 * 360
    private val readTimer = ElapsedTime()

    val velocity
        get(): Double {
            readTimer.reset()
            val currentPosition = analogInput.voltage / 3.3 * 360
            val velocity = (currentPosition - lastPosition) / readTimer.seconds()
            lastPosition = currentPosition
            return velocity
        }
}