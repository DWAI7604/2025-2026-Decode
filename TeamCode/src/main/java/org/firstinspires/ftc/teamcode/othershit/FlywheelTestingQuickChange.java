package org.firstinspires.ftc.teamcode.othershit;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FlywheelTestingNew - SINE", group="Linear OpMode")
@Configurable
public class FlywheelTestingQuickChange extends LinearOpMode {

    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private ElapsedTime timer = new ElapsedTime();
    private double targetVelocity; // RPM
    public static double targetVAmp = 50;
    public static double targetVSpread = 10;
    public static double timePause = 30; // milliseconds
    public static double weight = 0.1;

    public static double pVal = 10;
    public static double iVal = 0;
    public static double dVal = 0;
    public static double fVal = 0;

    private final double TICKS_PER_ROTATION = 384.5;

    private DcMotorEx flywheel;

//    private double prevPos;
//    private double currPos;
    private ElapsedTime timeChange = new ElapsedTime();

//    public static double pConst = 1.5;

    @Override
    public void runOpMode () {

        telemetry.addData("status", "initialized");
        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        targetVelocity = 0;

        telemetry.update();

        waitForStart();

//        prevPos = flywheel.getCurrentPosition();
//        currPos = prevPos;

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
//                currPos = flywheel.getCurrentPosition();
//                telemetry.addData("currPos: ", currPos);

                velocity = ((1.0 - weight) * velocity) + weight * flywheel.getVelocity() / TICKS_PER_ROTATION * 60; // 60 seconds per min
                timeChange.reset();

//                prevPos = currPos;

                flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pVal, iVal, dVal, fVal));
            }

            telemetry.addData("velocity: ", velocity);

            targetVelocity = (Math.sin(timer.seconds() / targetVSpread) + 1) * targetVAmp;

            flywheel.setVelocity(targetVelocity * TICKS_PER_ROTATION / 60);

//            double error = targetVelocity - velocity;
//            double power = pConst * Math.sqrt(Math.abs(error)) * Math.signum(error);
//
//            telemetry.addData("power: ", power);
//
//            if (power > 0.5)
//                power = 0.5;
//            else if (power < -0.5)
//                power = -0.5;
//
//            flywheel.setPower(power);
//
//            telemetry.update();
            panelsTelemetry.getTelemetry().addData("target", targetVelocity);
            panelsTelemetry.getTelemetry().addData("current", velocity);
            panelsTelemetry.getTelemetry().addData("actual current", flywheel.getVelocity() / TICKS_PER_ROTATION * 60);
            panelsTelemetry.getTelemetry().addData("power", flywheel.getPower());
            panelsTelemetry.getTelemetry().update();

        }

    }

}
