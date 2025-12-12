package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.PoseStorage;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name="*TeleOp - RED")
@Configurable
public class TeleOpRedRR extends LinearOpMode {

    public static PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    private static double PI_CONST = 3.14159;

    private PinpointLocalizer localizer;
    private Pose2d pose;

    public static double blockerPosClosed = .59;
    public static double blockerPosOpened = .78;

    public static double launchTimerDelayToIntake_FAR = 400; //ms
    public static double launchTimerDelayToPassthrough_FAR = 10; //ms
    public static double launchTimerDelayToIntake_CLOSE = 250; //ms
    public static double launchTimerDelayToPassthrough_CLOSE = 5; //ms
    private static double launchTimerDelayToIntake = launchTimerDelayToIntake_FAR; //ms
    private static double launchTimerDelayToPassthrough = launchTimerDelayToPassthrough_FAR; //ms

    public static double limSwitchDelayToPassthrough = 250; // ms
    private ElapsedTime passthroughTimer = new ElapsedTime();

    public static double INTAKE_POWER = 1.0;
    public static double PASSTHROUGH_POWER_CLOSE = 1.0;
    public static double PASSTHROUGH_POWER_FAR = 0.5;
    private static double passthroughPower = PASSTHROUGH_POWER_FAR;

    public static double CLOSE_VEL = 960;
    public static double MED_VEL = 1180;
    public static double FAR_VEL = 1490;

    public static double CLOSE_ANG = 1;
    public static double MED_ANG = 0.3;
    public static double FAR_ANG = 0;
    
    public static double YAW_STRENGTH = 0.85;
    public static double AUTOAIM_STICK_STRENGTH_FAR = 0.14;
    public static double AUTOAIM_STICK_STRENGTH_CLOSE = 0.2;

    private double autoaim_stick_strength = AUTOAIM_STICK_STRENGTH_FAR;

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
    private ElapsedTime launchTimer = new ElapsedTime();

    private boolean blockerClosedPrimary = true;
    private boolean blockerClosedSecondary = true;
    private boolean launching = false;
    private boolean ballReady = false;
    private boolean firstPass = true;

    private DriveState driveState = DriveState.fieldCentric;
    private ShotLocation shotLocation = ShotLocation.far;

    private boolean a1Prev = false;
    private boolean b1Prev = false;
    private boolean x1Prev = false;
    private boolean y1Prev = false;
    private boolean a2Prev = false;
    private boolean b2Prev = false;
    private boolean x2Prev = false;
    private boolean y2Prev = false;
    private boolean dpadUp2Prev = false;
    private boolean dpadDown2Prev = false;
    private boolean dpadLeft2Prev = false;
    private boolean dpadRight2Prev = false;
    private boolean bumperR2Prev = false;
    private boolean bumperL2Prev = false;
    private boolean triggerR1Prev = false;

    public void runOpMode() {

        localizer = new PinpointLocalizer(hardwareMap, 72.0 / 36653.0, PoseStorage.currentPose);

        //Establish hardware properties for drive motors
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Establish hardware properties for other devices
        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(700, 3, 200, 0));

        passthrough = hardwareMap.get(DcMotor.class, "passthrough");
        passthrough.setDirection(DcMotor.Direction.REVERSE);
        passthrough.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blocker = hardwareMap.get(Servo.class, "blocker");

        angleAdjustServo = hardwareMap.get(Servo.class, "angleAdjust");

        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        waitForStart();

        launchTimer.reset();
        passthroughTimer.reset();

        while (opModeIsActive()) {

            boolean limitSwitchIsPressed = !limitSwitch.isPressed(); // because the limit switch is wired backwards

            localizer.update();
            pose = localizer.getPose();

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
                    autoaim_stick_strength = AUTOAIM_STICK_STRENGTH_CLOSE;
                    passthroughPower = PASSTHROUGH_POWER_CLOSE;
                    launchTimerDelayToIntake = launchTimerDelayToIntake_CLOSE;
                    launchTimerDelayToPassthrough = launchTimerDelayToPassthrough_CLOSE;
                    flywheelPower = 0.525;
                    flywheelTarVel = CLOSE_VEL;
                    angleAdjust = CLOSE_ANG;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            } else if (gamepad2.x != x2Prev) {
                x2Prev = gamepad2.x;

                if (x2Prev) { // MEDIUM
                    autoaim_stick_strength = AUTOAIM_STICK_STRENGTH_CLOSE;
                    passthroughPower = PASSTHROUGH_POWER_CLOSE;
                    launchTimerDelayToIntake = launchTimerDelayToIntake_CLOSE;
                    launchTimerDelayToPassthrough = launchTimerDelayToPassthrough_CLOSE;
                    flywheelPower = 0.60;
                    flywheelTarVel = MED_VEL;
                    angleAdjust = MED_ANG;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            } else if (gamepad2.y != y2Prev) {
                y2Prev = gamepad2.y;

                if (y2Prev) { // FAR
                    autoaim_stick_strength = AUTOAIM_STICK_STRENGTH_FAR;
                    passthroughPower = PASSTHROUGH_POWER_FAR;
                    launchTimerDelayToIntake = launchTimerDelayToIntake_FAR;
                    launchTimerDelayToPassthrough = launchTimerDelayToPassthrough_FAR;
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
                telemetry.addData("rumble2: ", "is rumbling");
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
//                    blocker.setPosition(blockerPosClosed);
                    blockerClosedPrimary = true;
                    launching = false;
                }
            }

            if (launching) {

                ballReady = false;

//                blocker.setPosition(blockerPosOpened);
                blockerClosedPrimary = false;

                if (launchTimer.milliseconds() >= launchTimerDelayToPassthrough) {
                    passthrough.setPower(passthroughPower);
                }

                if (launchTimer.milliseconds() >= launchTimerDelayToIntake) {
                    intake.setPower(INTAKE_POWER);
                }

            }

            blockerClosedSecondary = !gamepad2.left_bumper;

            if (!blockerClosedPrimary || !blockerClosedSecondary) {
                blocker.setPosition(blockerPosOpened);
            } else {
                blocker.setPosition(blockerPosClosed);
            }

            // ===== LIMIT SWITCH TURNING OFF PASSTHROUGH =====
            if (limitSwitchIsPressed && intakeState != IntakeState.reject) {
                ballReady = true;
            }

            if (!launching && ballReady) {

                if (firstPass) {
                    passthroughTimer.reset();
                }

                if (passthroughTimer.milliseconds() >= limSwitchDelayToPassthrough) {
                    passthrough.setPower(0);
                }

                firstPass = false;
            } else {
                firstPass = true;
            }

            // ===== INTAKE STATE CHANGING =====
            /*\
             * A toggles between intaking
             * B toggles between rejecting
            \*/

            if (triggerR1Prev != (gamepad1.right_trigger > 0.5)) {

                triggerR1Prev = gamepad1.right_trigger > 0.5;

                if (triggerR1Prev) {
                    if (intakeState == IntakeState.on) {
                        intake.setPower(0);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(INTAKE_POWER);
                        intakeState = IntakeState.on;

                        if (!ballReady) {
                            passthrough.setPower(passthroughPower);
                        }
                    }
                }

            }

            if (a1Prev != gamepad1.a) {

                a1Prev = gamepad1.a;

                if (a1Prev) {
                    if (intakeState == IntakeState.on) {
                        intake.setPower(0);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(INTAKE_POWER);
                        intakeState = IntakeState.on;

                        if (!ballReady) {
                            passthrough.setPower(passthroughPower);
                        }
                    }
                }

            } else if (b1Prev != gamepad1.b) {

                b1Prev = gamepad1.b;

                if (b1Prev) {
                    if (intakeState == IntakeState.reject) {
                        intake.setPower(0);
                        passthrough.setPower(passthroughPower);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(-INTAKE_POWER);
                        intakeState = IntakeState.reject;
                        passthrough.setPower(-passthroughPower);
                        ballReady = false;
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
                passthrough.setPower(passthroughPower);
            } else if (gamepad1.dpad_right) {
                passthrough.setPower(0);
            } else if (gamepad1.dpad_down) {
                passthrough.setPower(-passthroughPower);
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

            // ===== RESET POSITION OF ROBOT =====
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                localizer.setPose( new Pose2d(72 - 8, -72 + 30, -PI_CONST / 2.0) );
            }

            // ===== DRIVE THE ROBOT =====
            /*\
             * x button toggles between field centric and robot centric
             * y button toggles between field centric and auto aim
            \*/
            if (x1Prev != gamepad1.x) {

                x1Prev = gamepad1.x;

                if (x1Prev) { // this code only runs when the button is pressed, not released
                    if (driveState == DriveState.fieldCentric) {
                        driveState = DriveState.robotCentric;
                    } else {
                        driveState = DriveState.fieldCentric;
                    }
                }

            } else if (y1Prev != gamepad1.y) {

                y1Prev = gamepad1.y;

                if (y1Prev) { // this code only runs when the button is pressed, not released
                    if (driveState == DriveState.autoAim) {
                        driveState = DriveState.fieldCentric;
                    } else {
                        driveState = DriveState.autoAim;
                    }
                }

            }

            if (driveState == DriveState.autoAim) {
                gamepad1.rumble(0.3, 0.3, 50);
                telemetry.addData("rumble1: ", "is rumbling");
            }

            mecanumDrive();

//            telemetry.addData("Power", flywheelPower);
            telemetry.addData("heading", pose.heading.toDouble());
            telemetry.addData("angle of flywheel", angleAdjust);
            telemetry.addData("velocity of flywheel", flywheel.getVelocity());
            telemetry.addData("TARVEL of flywheel", flywheelTarVel);
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.update();

            panelsTelemetry.getTelemetry().addData("flywheel velocity", flywheel.getVelocity());
            panelsTelemetry.getTelemetry().addData("flywheel target", flywheelTarVel);
//            panelsTelemetry.getTelemetry().addData("feeder position", feeder.getPosition());
            panelsTelemetry.getTelemetry().update();
        }
    }

    private void mecanumDrive () {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double heading = pose.heading.toDouble();
        heading = heading - PI_CONST / 2.0; // adjusted for (hopefully) red's POV
        double yaw = 0;
        double axial = 0;
        double lateral = 0;

        if (driveState == DriveState.fieldCentric) {
            yaw = -gamepad1.right_stick_x;
            axial = (gamepad1.left_stick_y * Math.cos(heading) + gamepad1.left_stick_x * Math.sin(heading));
            lateral = (gamepad1.left_stick_y * Math.sin(heading) - gamepad1.left_stick_x * Math.cos(heading));
        } else if (driveState == DriveState.robotCentric) {
            yaw = -gamepad1.right_stick_x;
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
        } else if (driveState == DriveState.autoAim) {
            yaw = locateGoal() - autoaim_stick_strength * (-gamepad2.right_stick_x);
            yaw = Math.min(Math.max(-yaw, -1.2), 1.2);
            axial = (gamepad1.left_stick_y * Math.cos(heading) + gamepad1.left_stick_x * Math.sin(heading));
            lateral = (gamepad1.left_stick_y * Math.sin(heading) - gamepad1.left_stick_x * Math.cos(heading));
        }

        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);

        leftFrontPower  = (axial + lateral + yaw) / denominator;
        leftBackPower   = (axial - lateral + yaw) / denominator;
        rightFrontPower = (axial - lateral - yaw) / denominator;
        rightBackPower  = (axial + lateral - yaw) / denominator;

        // Fine-control scaler: more left trigger press -> slower robot
        // Assumption: user wants the trigger to reduce speed significantly but not disable it.
        // Map trigger [0..1] to scaler roughly [1.0..0.2], then clamp to [0.15, 1.0].
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

        telemetry.addData("heading", heading);
        telemetry.addData("speedScaler", speedScaler);
    } // updates motors according to gamepad + locateGoal

    public double locateGoal () { // returns the yaw value so that the robot turns to the RED goal

        double yaw = 0;

        double robotHeading = pose.heading.toDouble();

        telemetry.addData("robotHeading", robotHeading);

        double xDist = (-72 + 3) - pose.position.x;
        double yDist = (72 - 5) - pose.position.y;

        telemetry.addData("xDist", xDist);
        telemetry.addData("yDist", yDist);

        double goalHeading = Math.atan(yDist / xDist);

        if (xDist < 0) // because arctan only gives values for x > 0 section of unit circle
            goalHeading = PI_CONST + goalHeading;

        goalHeading = (goalHeading - PI_CONST) % (2.0 * PI_CONST);

        telemetry.addData("goalHeading", goalHeading);

        yaw = (robotHeading - goalHeading) * YAW_STRENGTH;

        return yaw;

    } // returns yaw for rotation to goal

    enum DriveState {
        fieldCentric, autoAim, robotCentric
    }

    enum IntakeState {
        off, on, reject
    }

    enum ShotLocation {
        far, medium, close
    }
}
