package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testCode1021 extends RobotLinearOpMode {
    private DcMotor flywheel;
    private Servo feeder;
    private double power;
    private CRServo transfer;
    private DcMotor intake;
    private DcMotor leftFrontDriveMotor;
    private DcMotor leftBackDriveMotor;
    private DcMotor rightFrontDriveMotor;
    private DcMotor rightBackDriveMotor;

    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;
    private double axial;
    private double lateral;
    private double yaw;

    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        feeder = hardwareMap.get(Servo.class, "feeder");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBack");


        power = 0;


        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.a) {
                power = .65;
            } else if (gamepad2.b) {
                power = 0;
            } else if (gamepad2.x) {
                power = .7;
            } else if (gamepad2.y) {
                power = .75;
            } else if (gamepad2.dpad_up) {
                power = .6;
            } else if (gamepad2.dpad_left) {
                power = .55;
            } else if (gamepad2.dpad_right) {
                power = .5;
            } else if (gamepad2.dpad_down) {
                power = .45;
            }

            if (gamepad2.right_bumper) {

                feeder.setPosition(.001);
            } else if (gamepad2.left_bumper) {

                feeder.setPosition(.22);
            }

            intake.setPower(gamepad1.right_trigger);
            if (gamepad1.dpad_up) {
                transfer.setPower(1);
            } else if (gamepad1.dpad_down) {
                transfer.setPower(-1);
            }


            leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBackDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            axial = gamepad1.left_stick_y;
            lateral = -gamepad1.left_stick_x;
            yaw = -gamepad1.right_stick_x;

            leftFrontPower = (axial + lateral + yaw);
            leftBackPower = (axial - lateral + yaw);
            rightFrontPower = (axial - lateral - yaw);
            rightBackPower = (axial + lateral - yaw);

            leftFrontDriveMotor.setPower(leftFrontPower);
            leftBackDriveMotor.setPower(leftBackPower);
            rightFrontDriveMotor.setPower(rightFrontPower);
            rightBackDriveMotor.setPower(rightBackPower);

            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel.setPower(power);

            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}