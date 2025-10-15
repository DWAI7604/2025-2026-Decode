package org.firstinspires.ftc.teamcode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FlywheelTesting", group="Linear OpMode")
public class FlywheelTesting extends LinearOpMode {

    private DcMotor flywheel;

    private double prevPos;
    private double currPos;
    private ElapsedTime timeChange = new ElapsedTime();

    private double pConst = 1;

    @Override
    public void runOpMode () {

        telemetry.addData("status", "initialized");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        double targetVelocity = 100;

        telemetry.update();

        waitForStart();

        prevPos = flywheel.getCurrentPosition();
        currPos = prevPos;

        timeChange.startTime();

        while( opModeIsActive() ) {

            telemetry.addData("status", "running");

            currPos = flywheel.getCurrentPosition();
            double velocity = (currPos - prevPos) / timeChange.time(); // ticks / second
            timeChange.reset();

            telemetry.addData("velocity: ", velocity);

            double error = targetVelocity - velocity;
            double power = pConst * Math.sqrt(Math.abs(error)) * Math.signum(error);

            telemetry.addData("power: ", power);

            if (power > 0.8)
                power = 0.8;
            else if (power < -0.8)
                power = -0.8;

            flywheel.setPower(power);

            telemetry.update();

            prevPos = currPos;

        }

    }

}
