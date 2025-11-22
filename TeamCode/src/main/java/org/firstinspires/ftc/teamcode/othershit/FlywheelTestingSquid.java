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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name="FlywheelTesting - squid change", group="Linear OpMode")
@Configurable
public class FlywheelTestingSquid extends LinearOpMode {

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

    public static double close = 100;
    public static double med = 200;
    public static double far = 300;

    private final double TICKS_PER_ROTATION = 384.5;

    private DcMotorEx flywheel;

//    private double prevPos;
//    private double currPos;
    private ElapsedTime timeChange = new ElapsedTime();

    public static double pConst = 1.5;

    private boolean useVel = true;

    private boolean b2Prev = false;
    private boolean a2Prev = false;
    private boolean x2Prev = false;
    private boolean y2Prev = false;
    private boolean dpadUp2Prev = false;
    private boolean dpadDown2Prev = false;
    private boolean bumperR2Prev = false;
    private boolean bumperL2Prev = false;

    private double flywheelPower = 0;
    private double flywheelTarVel = 0;

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

            boolean updateFlywheel = false;
            if (gamepad2.b != b2Prev) {
                b2Prev = gamepad2.b;

                if (b2Prev) { // OFF
                    targetVelocity = 0;
                    updateFlywheel = true;
                }
            } else if (gamepad2.a != a2Prev) {
                a2Prev = gamepad2.a;

                if (a2Prev) { // CLOSE
                    targetVelocity = close;
                    updateFlywheel = true;
                }
            } else if (gamepad2.x != x2Prev) {
                x2Prev = gamepad2.x;

                if (x2Prev) { // MEDIUM
                    targetVelocity = med;
                    updateFlywheel = true;
                }
            } else if (gamepad2.y != y2Prev) {
                y2Prev = gamepad2.y;

                if (y2Prev) { // FAR
                    targetVelocity = far;
                    updateFlywheel = true;
                }
            }

            if (timeChange.milliseconds() > timePause) {
//                currPos = flywheel.getCurrentPosition();
//                telemetry.addData("currPos: ", currPos);

                velocity = ((1.0 - weight) * velocity) + weight * flywheel.getVelocity() / TICKS_PER_ROTATION * 60; // 60 seconds per min
                timeChange.reset();

//                prevPos = currPos;

//                flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pVal, iVal, dVal, fVal));
            }

            telemetry.addData("velocity: ", velocity);

//            targetVelocity = (Math.sin(timer.seconds() / targetVSpread) + 1) * targetVAmp;

//            flywheel.setVelocity(targetVelocity * TICKS_PER_ROTATION / 60);

            double error = targetVelocity - velocity;
            double power = pConst * Math.sqrt(Math.abs(error)) * Math.signum(error);
            panelsTelemetry.getTelemetry().addData("power uncapped", power);

            if (power > 0.8)
                power = 0.8;
            else if (power < -0.8)
                power = -0.8;

            flywheel.setPower(power);
//
//            telemetry.update();
            panelsTelemetry.getTelemetry().addData("target", targetVelocity);
            panelsTelemetry.getTelemetry().addData("current", velocity);
            panelsTelemetry.getTelemetry().addData("error", error);
            panelsTelemetry.getTelemetry().addData("actual current", flywheel.getVelocity() / TICKS_PER_ROTATION * 60);
            panelsTelemetry.getTelemetry().addData("power", power);
            panelsTelemetry.getTelemetry().update();

        }

    }

}
