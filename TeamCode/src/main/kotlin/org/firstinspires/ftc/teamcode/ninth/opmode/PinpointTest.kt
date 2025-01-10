package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.util.hardware.GoBildaPinpointDriver
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name = "Pinpoint Test")
class PinpointTest : LinearOpMode() {
    override fun runOpMode() {
        val pinpoint = hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint")
        pinpoint.setOffsets(0.0, 0.0)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD)
        pinpoint.resetPosAndIMU()

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset());
        telemetry.addData("Y offset", pinpoint.getYOffset());
        telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        telemetry.addData("Device Scalar", pinpoint.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {
            pinpoint.update();
            val pos = pinpoint.getPosition();
            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("x", pos.getX(DistanceUnit.INCH))
            telemetry.addData("y", pos.getY(DistanceUnit.INCH))
            telemetry.addData("heading", pos.getHeading(AngleUnit.DEGREES))
            telemetry.update()
        }
    }
}
