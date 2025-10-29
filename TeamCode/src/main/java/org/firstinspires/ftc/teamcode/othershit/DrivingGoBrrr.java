package org.firstinspires.ftc.teamcode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name="DrivingGoBrrr", group="Linear OpMode")
public class DrivingGoBrrr extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

//    private DcMotor flywheel;
//    private DcMotor intakeMotor;
//
//    private CRServo intakeWheel;
//    private Servo lever;
//
//    private boolean launching = false;
//    private boolean intaking = false;
//
//    private ElapsedTime launchTimer = new ElapsedTime();
//    private ElapsedTime intakeTimer = new ElapsedTime();

    @Override
    public void runOpMode () {
        telemetry.addData("status", "initialized");
        telemetry.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        flywheel = hardwareMap.get(DcMotor.class, "launchMotor");
//        intakeMotor = hardwareMap.get(DcMotor.class, "feedMotor");
//
//        intakeWheel = hardwareMap.get(CRServo.class, "intakeServo");
//        lever = hardwareMap.get(Servo.class, "feedServo");

        waitForStart();

//        launchTimer.reset();
//        intakeTimer.reset();

        while(opModeIsActive()) {
            telemetry.addData("status", "running");

            // MECANUM DRIVE
            mecanumDrive();

            // =========== SUBSYSTEMS ================

            // launch - a button
//            if (gamepad1.a && launchTimer.milliseconds() > 500) {
//
//                if(launching) {
//                    flywheel.setPower(0);
//                    launching = false;
//                } else {
//                    flywheel.setPower(-0.1);
//                    launching = true;
//                }
//
//                launchTimer.reset();
//                launchTimer.startTime();
//            }
//
//            // lever - b button
//            if (gamepad1.b) {
//                lever.setPosition(.15);
//            }
//            if (gamepad1.x) {
//                lever.setPosition(.85);
//            }
//            // intake wheel - right d
//            if (gamepad1.dpad_right) {
//                intakeWheel.setPower(-1);
//            }
//            if (gamepad1.dpad_left) {
//                intakeWheel.setPower(0);
//            }
//            // intake chain - up d
//            if (gamepad1.y && intakeTimer.milliseconds() > 500) {
//
//                if(intaking) {
//                    intakeMotor.setPower(0);
//                    intaking = false;
//                } else {
//                    intakeMotor.setPower(-0.1);
//                    intaking = true;
//                }
//
//                intakeTimer.reset();
//                intakeTimer.startTime();
//            }

            telemetry.update();
        }
    }

    private void mecanumDrive () {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        // controller
        double axial = gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        telemetry.addData("controller", String.format("%.2f : %.2f : %.2f", axial, lateral, yaw));

        leftFrontPower  = axial - lateral - yaw;
        leftBackPower   = axial + lateral - yaw;
        rightFrontPower = axial + lateral + yaw;
        rightBackPower  = axial - lateral + yaw;

        telemetry.addData("motors", String.format("%.2f : %.2f : %.2f : %.2f",
                leftFrontPower, leftBackPower, rightFrontPower, rightBackPower));

        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }
}