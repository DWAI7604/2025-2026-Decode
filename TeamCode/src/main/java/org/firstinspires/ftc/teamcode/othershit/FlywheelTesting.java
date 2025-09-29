package org.firstinspires.ftc.teamcode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="FlywheelTesting", group="Linear OpMode")
public class FlywheelTesting extends LinearOpMode {

    private DcMotor flywheel;

    @Override
    public void runOpMode () {

        telemetry.addData("status", "initialized");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        telemetry.update();

        waitForStart();

        while( opModeIsActive() ) {

            telemetry.addData("status", "running");

            double power = 1.0;

            telemetry.addData("power", power);

            flywheel.setPower(power);

            telemetry.update();

        }

    }

}
