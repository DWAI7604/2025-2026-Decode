package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name="FlywheelTele", group="Linear OpMode")
public class FlywheelTele extends LinearOpMode {
    private DcMotor flywheel;
    private FlywheelState flywheelState = FlywheelState.off;
    private FlywheelState flywheelStatePrev = FlywheelState.off;

    private double targetFlywheelVel = 0.0;
    private double flywheelVel = 0.0;

    @Override
    public void runOpMode () {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        waitForStart();

        while (opModeIsActive()) {
            // === update state based on controller ===

            if (gamepad1.b) {
                flywheelState = FlywheelState.off;
            } else if (gamepad1.a) {
                flywheelState = FlywheelState.close;
            } else if (gamepad1.x) {
                flywheelState = FlywheelState.middle;
            } else if (gamepad1.y) {
                flywheelState = FlywheelState.far;
            }

            // === adjust the flywheel ===

            if (flywheelState != flywheelStatePrev) {

            }
        }
    }

    enum FlywheelState {
        off, close, middle, far
    }
}
