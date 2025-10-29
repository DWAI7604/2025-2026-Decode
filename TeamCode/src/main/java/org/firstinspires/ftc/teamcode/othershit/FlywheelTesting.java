package org.firstinspires.ftc.teamcode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name="FlywheelTesting", group="Linear OpMode")
@Configurable
public class FlywheelTesting extends LinearOpMode {

    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private ElapsedTime timer = new ElapsedTime();
    private double targetVelocity;
    public static double targetVAmp = -0.5;
    public static double targetVSpread = 3;
    public static double timePause = 30;
    public static double weight = 0.1;

//    public static double feederPos = 0.0;
//    private Servo feeder;
//    public static boolean feederOn = false;

//    private static double TICKS_PER_ROTATION =

    private DcMotorEx flywheel;

    private double prevPos;
    private double currPos;
    private ElapsedTime timeChange = new ElapsedTime();

    public static double pConst = 1.5;

    @Override
    public void runOpMode () {

        telemetry.addData("status", "initialized");
        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");
//        feeder = hardwareMap.get(Servo.class, "feeder");

        targetVelocity = 0;

        telemetry.update();

        waitForStart();

        prevPos = flywheel.getCurrentPosition();
        currPos = prevPos;

        timeChange.startTime();
        double velocity = 0;

        while( opModeIsActive() ) {

            telemetry.addData("status", "running");

//            telemetry.addData("servo: ", feeder.getPosition());
//
//            if (feederOn) {
//                feeder.setPosition(feederPos);
//            }

            if (timeChange.milliseconds() > timePause) {
                currPos = flywheel.getCurrentPosition();
                telemetry.addData("currPos: ", currPos);

                velocity = ((1.0 - weight) * velocity) + weight * flywheel.getVelocity(AngleUnit.DEGREES) / 360.0 * 60.0; // rotations / min
                timeChange.reset();

                prevPos = currPos;
            }

            telemetry.addData("velocity: ", velocity);

            targetVelocity = (Math.sin(timer.seconds() / targetVSpread) + 1) * targetVAmp;

            double error = targetVelocity - velocity;
            double power = pConst * Math.sqrt(Math.abs(error)) * Math.signum(error);

            telemetry.addData("power: ", power);

            if (power > 0.5)
                power = 0.5;
            else if (power < -0.5)
                power = -0.5;

            flywheel.setPower(power);

            telemetry.update();
            panelsTelemetry.getTelemetry().addData("target", targetVelocity);
            panelsTelemetry.getTelemetry().addData("current", velocity);
            panelsTelemetry.getTelemetry().addData("power", power);
//            panelsTelemetry.getTelemetry().addData("servo", feeder.getPosition());
            panelsTelemetry.getTelemetry().update();

        }

    }

}
