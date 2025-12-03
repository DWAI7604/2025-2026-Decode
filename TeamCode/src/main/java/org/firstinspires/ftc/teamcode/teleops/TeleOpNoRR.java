package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;


@TeleOp(name="TeleOp - NO AUTOAIM / HEADLESS", group="Linear OpMode")
@Configurable
public class TeleOpNoRR extends LinearOpMode {

    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    public static double blockerPosClosed = .59;
    public static double blockerPosOpened = .78;

    public static double launchTimerDelayToIntake = 1000; //ms
    public static double launchTimerDelayToPassthrough = 100; //ms

    public static double INTAKE_POWER = 1.0;
    public static double PASSTHROUGH_POWER = 0.5;

    public static double CLOSE_VEL = 1000;
    public static double MED_VEL = 1360;
    public static double FAR_VEL = 1580;

    public static double CLOSE_ANG = 1;
    public static double MED_ANG = 0;
    public static double FAR_ANG = 0;

    private DcMotor leftFrontDriveMotor;
    private DcMotor leftBackDriveMotor;
    private DcMotor rightFrontDriveMotor;
    private DcMotor rightBackDriveMotor;

    private DcMotor intake;
    private DcMotor passthrough;
    private DcMotorEx flywheel;
    private Servo blocker;
    private Servo angleAdjustServo;
    private TouchSensor limitSwitch;

    private double flywheelPower;
    private double flywheelTarVel = 0;
    private double angleAdjust;
    private IntakeState intakeState = IntakeState.off;
    private LaunchState launchState = LaunchState.down;
    private ElapsedTime launchTimer = new ElapsedTime();

    private boolean launching = false;

    private boolean a1Prev = false;
    private boolean b1Prev = false;
    private boolean a2Prev = false;
    private boolean b2Prev = false;
    private boolean x2Prev = false;
    private boolean y2Prev = false;
    private boolean dpadUp2Prev = false;
    private boolean dpadDown2Prev = false;
    private boolean dpadLeft2Prev = false;
    private boolean dpadRight2Prev = false;
    private boolean bumperR2Prev = false;


    public void runOpMode() {

        //Establish hardware properties for drive motors
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Establish hardware properties for other devices
        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(800, 0.2, 0, 0));

        passthrough = hardwareMap.get(DcMotor.class, "passthrough");
        passthrough.setDirection(DcMotorSimple.Direction.REVERSE);
        passthrough.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blocker = hardwareMap.get(Servo.class, "blocker");

        angleAdjustServo = hardwareMap.get(Servo.class, "angleAdjust");

        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        waitForStart();

        launchTimer.reset();

        while (opModeIsActive()) {

            boolean limitSwitchIsPressed = !limitSwitch.isPressed(); // because the limit switch is wired backwards

            // ===== FLYWHEEL POWER & PRESETS =====
            /*\
             * B turns off flywheel
             * A sets flywheel to close range power
             * X sets flywheel to medium range power
             * Y sets flywheel to far range power
             * dpad up increases flywheel power by 0.05
             * dpad down decreases flywheel power by 0.05
            \*/
            boolean updateFlywheel = false;
            if (gamepad2.b != b2Prev) {
                b2Prev = gamepad2.b;

                if (b2Prev) { // OFF
                    flywheelPower = 0.00;
                    flywheelTarVel = 0;
                    updateFlywheel = true;
                }
            } else if (gamepad2.a != a2Prev) {
                a2Prev = gamepad2.a;

                if (a2Prev) { // CLOSE
                    flywheelPower = 0.525;
                    flywheelTarVel = CLOSE_VEL;
                    angleAdjust = CLOSE_ANG;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            } else if (gamepad2.x != x2Prev) {
                x2Prev = gamepad2.x;

                if (x2Prev) { // MEDIUM
                    flywheelPower = 0.60;
                    flywheelTarVel = MED_VEL;
                    angleAdjust = MED_ANG;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            } else if (gamepad2.y != y2Prev) {
                y2Prev = gamepad2.y;

                if (y2Prev) { // FAR
                    flywheelPower = 0.725;
                    flywheelTarVel = FAR_VEL;
                    angleAdjust = FAR_ANG;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            }

            if (gamepad2.dpad_up != dpadUp2Prev) {
                dpadUp2Prev = gamepad2.dpad_up;

                if (dpadUp2Prev) {
                    flywheelPower += 0.025;
                    flywheelTarVel += 10;
                    updateFlywheel = true;
                }
            } else if (gamepad2.dpad_down != dpadDown2Prev) {
                dpadDown2Prev = gamepad2.dpad_down;

                if (dpadDown2Prev) {
                    flywheelPower -= 0.025;
                    flywheelTarVel -= 10;
                    updateFlywheel = true;
                }
            }

            if (updateFlywheel) {
//                flywheelPower = Math.max(Math.min(flywheelPower, 0.85), 0); // caps flywheel from going above 85% or into reverse
//                flywheel.setPower(flywheelPower);

                flywheel.setVelocity(flywheelTarVel);
            }

            if (Math.abs(flywheel.getVelocity() - flywheelTarVel) > 20) {
                gamepad2.rumble(0.5, 0.5, 50);
                telemetry.addData("rumble: ", "is rumbling");
            }

            // ===== LAUNCH =====
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
                    passthrough.setPower(0);
                    blocker.setPosition(blockerPosClosed);
                    launching = false;
                }
            }

            if (launching) {

                blocker.setPosition(blockerPosOpened);

                if (launchTimer.milliseconds() >= launchTimerDelayToPassthrough) {
                    passthrough.setPower(PASSTHROUGH_POWER);
                }

                if (launchTimer.milliseconds() >= launchTimerDelayToIntake) {
                    intake.setPower(INTAKE_POWER);
                }

            }

            // ===== LIMIT SWITCH TURNING OFF PASSTHROUGH =====
            if (!launching && limitSwitchIsPressed) {
                passthrough.setPower(0);
            }

            // ===== INTAKE STATE CHANGING =====
            /*\
             * A toggles between intaking
             * B toggles between rejecting
            \*/
            if (a1Prev != gamepad1.a) {

                a1Prev = gamepad1.a;

                if (a1Prev) {
                    if (intakeState == IntakeState.on) {
                        intake.setPower(0);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(INTAKE_POWER);
                        intakeState = IntakeState.on;

                        if (!limitSwitchIsPressed) {
                            passthrough.setPower(PASSTHROUGH_POWER);
                        }
                    }
                }

            } else if (b1Prev != gamepad1.b) {

                b1Prev = gamepad1.b;

                if (b1Prev) {
                    if (intakeState == IntakeState.reject) {
                        intake.setPower(0);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(-INTAKE_POWER);
                        intakeState = IntakeState.reject;
                    }
                }


            }

            // ===== PASSTHROUGH =====
            /*\
             * dpad up brings it into robot
             * dpad right pauses it
             * dpad down rejects the artifact
            \*/
            if (gamepad1.dpad_up) {
                passthrough.setPower(PASSTHROUGH_POWER);
            } else if (gamepad1.dpad_right) {
                passthrough.setPower(0);
            } else if (gamepad1.dpad_down) {
                passthrough.setPower(-PASSTHROUGH_POWER);
            }

            // ===== ADJUST ANGLE OF FLYWHEEL =====
            /*\
             * dpad left increases the launch angle
             * dpad right decreases it
            \*/
            boolean updateAngleOfFlywheel = false;
            if (gamepad2.dpad_left != dpadLeft2Prev) {
                dpadLeft2Prev = gamepad2.dpad_left;

                if (dpadLeft2Prev) {
                    angleAdjust += 0.05;
                    updateAngleOfFlywheel = true;
                }
            } else if (gamepad2.dpad_right != dpadRight2Prev) {
                dpadRight2Prev = gamepad2.dpad_right;

                if (dpadRight2Prev) {
                    angleAdjust -= 0.05;
                    updateAngleOfFlywheel = true;
                }
            }
            if (updateAngleOfFlywheel) {
                angleAdjust = Math.max(Math.min(angleAdjust, 1.0), 0);

                angleAdjustServo.setPosition(angleAdjust);
            }

            // ===== DRIVE THE ROBOT =====
            mecanumDrive();

            telemetry.addData("Power", flywheelPower);
            telemetry.addData("angle of flywheel", angleAdjust);
            telemetry.addData("tar vel", flywheelTarVel);
            telemetry.addData("actual vel", flywheel.getVelocity());
            telemetry.update();
        }
    }

    private void mecanumDrive () {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double yaw = 0;
        double axial = 0;
        double lateral = 0;

        yaw = -gamepad1.right_stick_x;
        axial = gamepad1.left_stick_y;
        lateral = -gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);

        leftFrontPower  = (axial + lateral + yaw) / denominator;
        leftBackPower   = (axial - lateral + yaw) / denominator;
        rightFrontPower = (axial - lateral - yaw) / denominator;
        rightBackPower  = (axial + lateral - yaw) / denominator;

        double speedScaler = 1.0 - 0.8 * gamepad1.left_trigger; // full trigger -> ~0.2
        speedScaler = Math.max(0.15, Math.min(speedScaler, 1.0));

        leftFrontPower  *= speedScaler;
        leftBackPower   *= speedScaler;
        rightFrontPower *= speedScaler;
        rightBackPower  *= speedScaler;

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);
    } // updates motors according to gamepad + locateGoal

    enum IntakeState {
        off, on, reject
    }

    enum LaunchState {
        up, down
    }
}
