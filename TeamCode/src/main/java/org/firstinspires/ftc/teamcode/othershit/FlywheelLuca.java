package org.firstinspires.ftc.teamcode.othershit;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Flywheel - LUCA", group="Linear OpMode")
@Configurable
public class FlywheelLuca extends LinearOpMode {

    private DcMotorEx flywheel;

    private double flywheelPower;

    @Override
    public void runOpMode () {

        telemetry.addData("status", "initialized");

        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.update();

        waitForStart();

        while( opModeIsActive() ) {

            telemetry.addData("status", "running");

            if (gamepad1.b) {
                flywheelPower = 0;
            } else if (gamepad1.a) {
                flywheelPower = 0.50;
            } else if (gamepad1.x) {
                flywheelPower = 0.65;
            } else if (gamepad1.y) {
                flywheelPower = 0.80;
            }

            flywheel.setPower(flywheelPower);

            telemetry.addData("flywheel power", flywheelPower);
            telemetry.addLine("------------");
            telemetry.addLine("b = 0.00");
            telemetry.addLine("a = 0.50");
            telemetry.addLine("x = 0.65");
            telemetry.addLine("y = 0.80");
            telemetry.update();

        }

    }
}