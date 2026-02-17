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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//@Disabled
@TeleOp(name="Double Flywheel", group="Linear OpMode")
@Configurable
public class DoubleFlywheel extends LinearOpMode {

    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private ElapsedTime timer = new ElapsedTime();
    public static double targetVAmp = 500;
    public static double targetVSpread = 3;
    public static double timePause = 50; // milliseconds
    public static double weight = 0.1;

    public static double tune0_pVal = 0; // 170
    public static double tune1_iVal = 0; // 0.15
    public static double tune2_dVal = 0;
    public static double tune3_fVal = 0;

    public static double tune0_pVal_diff = 0;
    public static double tune1_iVal_diff = 0;
    public static double tune2_dVal_diff = 0;
    public static double tune3_fVal_diff = 0;

    public static int flywheel_step = 0;

    public static double flywheel_step_diff = 100; // inc vel
    public static double flywheel_step_pause = 500; // ms

    private ElapsedTime flywheelLowerTimer = new ElapsedTime();
    private boolean flywheelLowering = false;
    private double flywheelTempVel = 0;

    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    private ElapsedTime timeChange = new ElapsedTime();

    private static double blockerPosClosed = .11;
    private static double blockerPosOpened = .52;

    private static double launchTimerDelayToIntake = 10; //ms
    private static double launchTimerDelayToPassthrough = 10; //ms

    private ElapsedTime passthroughTimer = new ElapsedTime();

    private static double INTAKE_POWER = 1.0;
    private static double PASSTHROUGH_POWER = 0.75;

//    private static double CLOSE_VEL = 1000;
//    private static double MED_VEL = 1200;
//    private static double FAR_VEL = 1400;
//
//    private static double CLOSE_ANG = 1.00;
//    private static double MED_ANG = 0.75;
//    private static double FAR_ANG = 0.00;

    private DcMotor intake;
    private DcMotor passthrough;
    private Servo blocker;
    private Servo angleAdjustServo;
    private TouchSensor limitSwitch;

    private double flywheelPower;
    public static double flywheelTarVel = 0;
    public static double angleAdjust = 0;
    private ElapsedTime launchTimer = new ElapsedTime();

    private boolean launching = false;

    private boolean a2Prev = false;
    private boolean b2Prev = false;
    private boolean x2Prev = false;
    private boolean y2Prev = false;
    private boolean dpadUp2Prev = false;
    private boolean dpadDown2Prev = false;
    private boolean dpadLeft2Prev = false;
    private boolean dpadRight2Prev = false;
    private boolean bumperR2Prev = false;

    @Override
    public void runOpMode() {

        telemetry.addData("status", "initialized");

        flywheel1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel2");
//        flywheel2.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        passthrough = hardwareMap.get(DcMotor.class, "passthrough");
//        passthrough.setDirection(DcMotor.Direction.REVERSE);
//        passthrough.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blocker = hardwareMap.get(Servo.class, "blocker");
//
        angleAdjustServo = hardwareMap.get(Servo.class, "angleAdjust");

        double velocity = 0;

        telemetry.update();

        waitForStart();

        flywheelTempVel = flywheel1.getVelocity();

        launchTimer.reset();
        passthroughTimer.reset();
//        timeChange.reset();

        while (opModeIsActive()) {

            telemetry.addData("status", "running");

            if (timeChange.milliseconds() > timePause) {

                timeChange.reset();

                flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(tune0_pVal, tune1_iVal, tune2_dVal, tune3_fVal));
                flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(tune0_pVal + tune0_pVal_diff, tune1_iVal + tune1_iVal_diff, tune2_dVal + tune2_dVal_diff, tune3_fVal + tune3_fVal_diff));

                angleAdjustServo.setPosition(angleAdjust);

            }

            if (flywheelTempVel != flywheelTarVel && flywheelLowerTimer.milliseconds() > flywheel_step_pause) {

                double error = flywheelTarVel - flywheelTempVel;

                if (Math.abs(error) < flywheel_step_diff)
                    flywheelTempVel = flywheelTarVel;

                else if (error > 0)
                    flywheelTempVel += flywheel_step_diff;

                else if (error < 0)
                    flywheelTempVel -= flywheel_step_diff;

                flywheelLowerTimer.reset();

            }

            flywheel1.setVelocity(flywheelTempVel);
            flywheel2.setVelocity(flywheelTempVel);


            // ===== FLYWHEEL POWER & PRESETS =====
            /*\
             * B turns off flywheel
             * A sets flywheel to close range power
             * X sets flywheel to medium range power
             * Y sets flywheel to far range power
             * dpad up increases flywheel power by 0.05
             * dpad down decreases flywheel power by 0.05
            \*/
//            boolean updateFlywheel = false;
//            if (gamepad2.b != b2Prev) {
//                b2Prev = gamepad2.b;
//
//                if (b2Prev) { // OFF
//                    flywheelPower = 0.00;
//                    flywheelTarVel = 0;
//                    updateFlywheel = true;
//                }
//            } else if (gamepad2.a != a2Prev) {
//                a2Prev = gamepad2.a;
//
//                if (a2Prev) { // CLOSE
//                    if (flywheelTarVel > CLOSE_VEL) {
////                        flywheelLowering = true;
//                        flywheelLowerTimer.reset();
//                        flywheel_step = 1;
//                    }
//                    flywheelPower = 0.525;
//                    flywheelTarVel = CLOSE_VEL;
//                    angleAdjust = CLOSE_ANG;
//                    angleAdjustServo.setPosition(angleAdjust);
//                    updateFlywheel = true;
//                }
//            } else if (gamepad2.x != x2Prev) {
//                x2Prev = gamepad2.x;
//
//                if (x2Prev) { // MEDIUM
//                    if (flywheelTarVel > MED_VEL) {
////                        flywheelLowering = true;
//                        flywheelLowerTimer.reset();
//                        flywheel_step = 1;
//                    }
//                    flywheelPower = 0.60;
//                    flywheelTarVel = MED_VEL;
//                    angleAdjust = MED_ANG;
//                    angleAdjustServo.setPosition(angleAdjust);
//                    updateFlywheel = true;
//                }
//            } else if (gamepad2.y != y2Prev) {
//                y2Prev = gamepad2.y;
//
//                if (y2Prev) { // FAR
//                    if (flywheelTarVel > FAR_VEL) {
////                        flywheelLowering = true;
//                        flywheelLowerTimer.reset();
//                        flywheel_step = 1;
//                    }
//                    flywheelPower = 0.725;
//                    flywheelTarVel = FAR_VEL;
//                    angleAdjust = FAR_ANG;
//                    angleAdjustServo.setPosition(angleAdjust);
//                    updateFlywheel = true;
//                }
//            }
//
//            if (gamepad2.dpad_up != dpadUp2Prev) {
//                dpadUp2Prev = gamepad2.dpad_up;
//
//                if (dpadUp2Prev) {
//                    flywheelPower += 0.025;
//                    flywheelTarVel += 20;
//                    updateFlywheel = true;
//                }
//            } else if (gamepad2.dpad_down != dpadDown2Prev) {
//                dpadDown2Prev = gamepad2.dpad_down;
//
//                if (dpadDown2Prev) {
//                    flywheelPower -= 0.025;
//                    flywheelTarVel -= 20;
//                    updateFlywheel = true;
//                }
//            }

//            if (updateFlywheel || flywheelLowering) {
////                flywheelPower = Math.max(Math.min(flywheelPower, 0.85), 0); // caps flywheel from going above 85% or into reverse
////                flywheel.setPower(flywheelPower);
//
//                if (flywheelLowering) {
//
//                    if (flywheel_step == 1) {
//                        velocity = flywheel1.getVelocity();
//                        velocity = flywheel2.getVelocity();
//                    }
//
//                    if (flywheel_step > flywheel_step_max) {
//
//                        flywheelLowering = false;
//                        flywheel_step++;
//                        flywheel1.setVelocity(flywheelTarVel);
//                        flywheel2.setVelocity(flywheelTarVel);
//
//                    } else if (flywheelLowerTimer.milliseconds() > flywheel_step_diff) {
//
//                        velocity = (flywheelTarVel + velocity * (flywheel_step_max - flywheel_step)) / (flywheel_step_max + 1 - flywheel_step);
//
//                        flywheel1.setVelocity(velocity);
//                        flywheel2.setVelocity(velocity);
//                        flywheel_step++;
//                        flywheelLowerTimer.reset();
//
//                    }
//
////                    if (flywheelLowering && flywheelLowerTimer.milliseconds() < flywheel_step_1 && flywheel_step == 1) {
////                        velocity = (flywheelTarVel + flywheel.getVelocity() + flywheel.getVelocity() + flywheel.getVelocity()) / 4.0;
////                        flywheel.setVelocity(velocity);
////                        flywheel_step++;
////                    } else if (flywheelLowering && flywheelLowerTimer.milliseconds() > flywheel_step_1 && flywheel_step == 2) {
////                        velocity = (flywheelTarVel + velocity + velocity) / 3.0;
////                        flywheel.setVelocity(velocity);
////                        flywheel_step++;
////                    } else if (flywheelLowering && flywheelLowerTimer.milliseconds() > flywheel_step_2 && flywheel_step == 3) {
////                        velocity = (flywheelTarVel + velocity) / 2.0;
////                        flywheel.setVelocity(velocity);
////                        flywheel_step++;
////                    } else if (flywheelLowering && flywheelLowerTimer.milliseconds() > flywheel_step_3) {
////                    }
//
//                } else {
//                    flywheel1.setVelocity(flywheelTarVel);
//                    flywheel2.setVelocity(flywheelTarVel);
//                }
//            }

//            // ===== ADJUST ANGLE OF FLYWHEEL =====
//            /*\
//             * dpad left increases the launch angle
//             * dpad right decreases it
//            \*/
//            boolean updateAngleOfFlywheel = false;
//            if (gamepad2.dpad_left != dpadLeft2Prev) {
//                dpadLeft2Prev = gamepad2.dpad_left;
//
//                if (dpadLeft2Prev) {
//                    angleAdjust += 0.05;
//                    updateAngleOfFlywheel = true;
//                }
//            } else if (gamepad2.dpad_right != dpadRight2Prev) {
//                dpadRight2Prev = gamepad2.dpad_right;
//
//                if (dpadRight2Prev) {
//                    angleAdjust -= 0.05;
//                    updateAngleOfFlywheel = true;
//                }
//            }
//            if (updateAngleOfFlywheel) {
//                angleAdjust = Math.max(Math.min(angleAdjust, 1.0), 0);
//
//                angleAdjustServo.setPosition(angleAdjust);
//            }
//
            // ===== LAUNCHING MECHANISM FROM TELEOP =====
            /*\
             * when button pressed and held:
             * 1.) opens the blocker
             * 2.) runs passthrough
             * 3.) runs intake
             * when button released: stop passthrough + intake, close blocker
            \*/
            if (gamepad2.right_bumper != bumperR2Prev) {
                bumperR2Prev = gamepad2.right_bumper;

                if (bumperR2Prev) {
                    launchTimer.reset();
                    launching = true;
                } else {
                    intake.setPower(0);
//                    passthrough.setPower(0);
                    blocker.setPosition(blockerPosClosed);
                    launching = false;
                }
            }

            if (launching) {

                blocker.setPosition(blockerPosOpened);

//                if (launchTimer.milliseconds() >= launchTimerDelayToPassthrough) {
//                    passthrough.setPower(PASSTHROUGH_POWER);
//                }

                if (launchTimer.milliseconds() >= launchTimerDelayToIntake) {
                    intake.setPower(INTAKE_POWER);
                }

            }
//
            panelsTelemetry.getTelemetry().addData("target velocity", flywheelTarVel);
//            panelsTelemetry.getTelemetry().addData("hypothetical velocity", velocity);
            panelsTelemetry.getTelemetry().addData("actual velocity 1", flywheel1.getVelocity());
            panelsTelemetry.getTelemetry().addData("actual velocity 2", flywheel2.getVelocity());
//            panelsTelemetry.getTelemetry().addData("power", flywheel1.getPower());
            panelsTelemetry.getTelemetry().addData("voltage 1", flywheel1.getCurrent(CurrentUnit.AMPS));
            panelsTelemetry.getTelemetry().addData("voltage 2", flywheel2.getCurrent(CurrentUnit.AMPS));
            panelsTelemetry.getTelemetry().update();

        }

    }
}