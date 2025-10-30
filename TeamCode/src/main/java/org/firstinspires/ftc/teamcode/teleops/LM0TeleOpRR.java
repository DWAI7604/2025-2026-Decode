package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.PoseStorage;

@TeleOp(name="LM0TeleOp - RED", group="Linear OpMode")
public class LM0TeleOpRR extends LinearOpMode {

    private static double PI_CONST = 3.14159;

    private PinpointLocalizer localizer;
    private Pose2d pose;

    private DcMotor leftFrontDriveMotor;
    private DcMotor leftBackDriveMotor;
    private DcMotor rightFrontDriveMotor;
    private DcMotor rightBackDriveMotor;

    private DcMotor flywheel;
    private DcMotor intake;
    private Servo feeder; // this is that lil flicky switch
    private Servo angleAdjustServo;
    private CRServo transfer; // this is that rubber band thingy
    private Servo flipServo;

    private double flywheelPower;
    private double angleAdjust;
    private IntakeState intakeState = IntakeState.off;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private FeederState feederState = FeederState.down;
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime flywheelTimer = new ElapsedTime();
    private ElapsedTime angleTimer = new ElapsedTime();
    private ElapsedTime flipTimer = new ElapsedTime();
    private FlipState flipState = FlipState.up;
    private DriveState driveState = DriveState.fieldCentric;
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
    private boolean bumperL2Prev = false;
    private boolean bumperR2Prev = false;

    public void runOpMode() {

        localizer = new PinpointLocalizer(hardwareMap, 72.0 / 36653.0, PoseStorage.currentPose);

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
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder = hardwareMap.get(Servo.class, "feeder");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        angleAdjustServo = hardwareMap.get(Servo.class, "angleAdjust");
        flipServo = hardwareMap.get(Servo.class, "flap");

        waitForStart();

        intakeTimer.reset();
        feederTimer.reset();
        flywheelTimer.reset();
        angleTimer.reset();
        flipTimer.reset();

        while (opModeIsActive()) {

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
                    updateFlywheel = true;
                }
            } else if (gamepad2.a != a2Prev) {
                a2Prev = gamepad2.a;

                if (a2Prev) { // CLOSE
                    flywheelPower = 0.525;
                    angleAdjust = 1.0;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            } else if (gamepad2.x != x2Prev) {
                x2Prev = gamepad2.x;

                if (x2Prev) { // MEDIUM
                    flywheelPower = 0.60;
                    angleAdjust = 0.6;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            } else if (gamepad2.y != y2Prev) {
                y2Prev = gamepad2.y;

                if (y2Prev) { // FAR
                    flywheelPower = 0.70;
                    angleAdjust = 0.0;
                    angleAdjustServo.setPosition(angleAdjust);
                    updateFlywheel = true;
                }
            }

            if (gamepad2.dpad_up != dpadUp2Prev) {
                dpadUp2Prev = gamepad2.dpad_up;

                if (dpadUp2Prev) {
                    flywheelPower += 0.025;
                    updateFlywheel = true;
                }
            } else if (gamepad2.dpad_down != dpadDown2Prev) {
                dpadDown2Prev = gamepad2.dpad_down;

                if (dpadDown2Prev) {
                    flywheelPower -= 0.025;
                    updateFlywheel = true;
                }
            }

            if (updateFlywheel) {
                flywheelPower = Math.max(Math.min(flywheelPower, 0.85), 0); // caps flywheel from going above 85% or into reverse
                flywheel.setPower(flywheelPower);
            }

            // ===== FEEDER FLICKING =====
            /*\
             * right bumper will flick the ball up into the flywheel
            \*/

            if (feederTimer.milliseconds() > 150) {

                if (feederState == FeederState.up) {
                    feeder.setPosition(.22);
                    feederState = FeederState.down;
                } else if (gamepad2.right_bumper != bumperR2Prev) {
                    bumperR2Prev = gamepad2.right_bumper;

                    if (bumperR2Prev) {
                        feeder.setPosition(.001);
                        feederState = FeederState.up;
                        feederTimer.reset();
                    }
                }
            }

            // ===== INTAKE STATE CHANGING =====
            /*\
             * A toggles between intaking
             * B toggles between rejecting
            \*/
            if (a1Prev != gamepad1.a) {

                a1Prev = gamepad1.a;

                if (a1Prev) { // this code only runs when the a button is pressed, not released
                    if (intakeState == IntakeState.on) {
                        intake.setPower(0);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(1);
                        intakeState = IntakeState.on;
                    }
                }

            } else if (b1Prev != gamepad1.b) {

                b1Prev = gamepad1.b;

                if (b1Prev) { // this code only runs when the a button is pressed, not released
                    if (intakeState == IntakeState.reject) {
                        intake.setPower(0);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(-1);
                        intakeState = IntakeState.reject;
                    }
                }


            }

            // ===== TRANSFER INTO ROBOT =====
            /*\
             * dpad up brings it into robot
             * dpad right pauses it
             * dpad down rejects the artifact
            \*/
            if (gamepad1.dpad_up) {
                transfer.setPower(1);
            } else if (gamepad1.dpad_right) {
                transfer.setPower(0);
            } else if (gamepad1.dpad_down) {
                transfer.setPower(-1);
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
                    angleAdjust += 0.025;
                    angleTimer.reset();
                    updateAngleOfFlywheel = true;
                }
            } else if (gamepad2.dpad_right != dpadRight2Prev) {
                dpadRight2Prev = gamepad2.dpad_right;

                if (dpadRight2Prev) {
                    angleAdjust -= 0.025;
                    angleTimer.reset();
                    updateAngleOfFlywheel = true;
                }
            }
            if (updateAngleOfFlywheel) {
                angleAdjust = Math.max(Math.min(angleAdjust, 1.0), 0);

                angleAdjustServo.setPosition(angleAdjust);
            }

            // ===== FLIP THAT INTERNAL FLAP =====
            /*\
             * left bumper toggles it
            \*/
            if (gamepad2.left_bumper != bumperL2Prev) {
                bumperL2Prev = gamepad2.left_bumper;

                if (bumperL2Prev) {
                    if (flipState == FlipState.down) { // move up
                        flipState = FlipState.up;
                        flipServo.setPosition(0.0);
                    } else {
                        flipState = FlipState.down;
                        flipServo.setPosition(0.1);
                    }
                }
            }

            // ===== RESET POSITION OF ROBOT =====
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                localizer.setPose( new Pose2d(72 - 9, -72 + 30, -PI_CONST / 2.0) );
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
            mecanumDrive();

            telemetry.addData("Power", flywheelPower);
            telemetry.addData("heading", pose.heading.toDouble());
            telemetry.addData("angle of flywheel", angleAdjust);
            telemetry.addData("flipServo: ", flipState);
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.update();
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
            axial = gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
        } else if (driveState == DriveState.autoAim) {
            yaw = locateGoal();
            yaw = Math.min(Math.max(-yaw, -1.0), 1.0);
            axial = (gamepad1.left_stick_y * Math.cos(heading) + gamepad1.left_stick_x * Math.sin(heading));
            lateral = (gamepad1.left_stick_y * Math.sin(heading) - gamepad1.left_stick_x * Math.cos(heading));
        }

//        telemetry.addData("controller", String.format("%.2f : %.2f : %.2f", axial, lateral, yaw));

        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);

        leftFrontPower  = (axial + lateral + yaw) / denominator;
        leftBackPower   = (axial - lateral + yaw) / denominator;
        rightFrontPower = (axial - lateral - yaw) / denominator;
        rightBackPower  = (axial + lateral - yaw) / denominator;

//        telemetry.addData("motors", String.format("%.2f : %.2f : %.2f : %.2f",
//                leftFrontPower, leftBackPower, rightFrontPower, rightBackPower));

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);

        telemetry.addData("heading", heading);
    } // updates motors according to gamepad + locateGoal

    public double locateGoal () { // returns the yaw value so that the robot turns to the RED goal

        double yaw = 0;

        double robotHeading = pose.heading.toDouble();

        telemetry.addData("robotHeading", robotHeading);

        double xDist = (-72 + 16) - pose.position.x;
        double yDist = (72 - 5) - pose.position.y;

        telemetry.addData("xDist", xDist);
        telemetry.addData("yDist", yDist);

        double goalHeading = Math.atan(yDist / xDist);

        if (xDist < 0) // because arctan only gives values for x > 0 section of unit circle
            goalHeading = PI_CONST + goalHeading;

        goalHeading = (goalHeading - PI_CONST) % (2.0 * PI_CONST);

        telemetry.addData("goalHeading", goalHeading);

        yaw = (robotHeading - goalHeading) * .50;

        return yaw;

    } // returns yaw for rotation to goal

    enum DriveState {
        fieldCentric, autoAim, robotCentric
    }

    enum IntakeState {
        off, on, reject
    }

    enum FeederState {
        up, down
    }

    enum FlipState {
        up, down
    }
}
