package org.firstinspires.ftc.teamcode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp
public class TestingTeleOp extends LinearOpMode {

    DcMotor leftFrontDriveMotor;
    DcMotor leftBackDriveMotor;
    DcMotor rightFrontDriveMotor;
    DcMotor rightBackDriveMotor;
    DcMotor intake;
    DcMotor flywheel;

    Servo angleAdjust;
    CRServo passthrough;
    Servo blocker;

    TouchSensor limitSwitch;

    double axial;
    double lateral;
    double yaw;

    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    double limitSwitchCounter;
    double ballCount;
    boolean limitSwitchWasPressed = false;


    public void runOpMode() {

        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBackDriveMotor");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBackDriveMotor");

        leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        angleAdjust = hardwareMap.get(Servo.class, "angleAdjust");
        passthrough = hardwareMap.get(CRServo.class, "passthrough");
        blocker = hardwareMap.get(Servo.class, "blocker");

        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        ballCount = 0;

        waitForStart();
        while (opModeIsActive()) {

            //Gamepad1 commands
            axial = gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            leftFrontPower = axial - lateral + yaw;
            leftBackPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            rightBackPower = axial + lateral - yaw;

            leftFrontDriveMotor.setPower(leftFrontPower);
            leftBackDriveMotor.setPower(leftBackPower);
            rightFrontDriveMotor.setPower(rightFrontPower);
            rightBackDriveMotor.setPower(rightBackPower);

            if (gamepad1.a) {
                intake.setPower(1);

            } else if (gamepad1.b) {
                intake.setPower(-1);

            } else if (gamepad1.x) {
                intake.setPower(0);

            }

            //Gamepad2 commands
            if (gamepad2.a) {
                //open blocker
                blocker.setPosition(0.999);
            } else if (gamepad2.b) {
                //close blocker
                blocker.setPosition(0.6);
            }

            if (gamepad2.x) {
                flywheel.setPower(-0.85);
            } else if (gamepad2.y) {
                flywheel.setPower(0);
            }

            if (gamepad2.dpad_up) {
                angleAdjust.setPosition(0.5);
            } else if (gamepad2.dpad_down) {
                angleAdjust.setPosition(0.001);
            }

            if (gamepad2.right_bumper) {
                passthrough.setPower(-1);
            } else if (gamepad2.left_bumper) {
                passthrough.setPower(0);
            }



            if (!limitSwitch.isPressed() && !limitSwitchWasPressed) {
                ballCount++;
                limitSwitchWasPressed = true;
            } else if (limitSwitch.isPressed() && limitSwitchWasPressed) {
                limitSwitchWasPressed = false;
            }

            telemetry.addData("Balls Fired: ", ballCount);
            telemetry.addData("Limit Switch is Pressed:", limitSwitch.isPressed());
            telemetry.addData("limit switch was pressed", limitSwitchWasPressed);
            telemetry.update();

        }

    }
}
