package org.firstinspires.ftc.teamcode.teleops;

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

@TeleOp(name="LM0TeleOp - Zoe", group="Linear OpMode")
public class LM0TeleOp extends LinearOpMode {

    private DcMotor leftFrontDriveMotor;
    private DcMotor leftBackDriveMotor;
    private DcMotor rightFrontDriveMotor;
    private DcMotor rightBackDriveMotor;

    private DcMotor flywheel;
    private DcMotor intake;
    private Servo feeder; // this is that lil flicky switch
    private CRServo transfer; // what is this???????

    private GoBildaPinpointDriver odo;

    private double flywheelPower;
    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;
    private double axial;
    private double lateral;
    private double yaw;
    private double angleOffset;
    private IntakeState intakeState = IntakeState.off;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private FeederState feederState = FeederState.down;
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime flywheelTimer = new ElapsedTime();
    private boolean b1Pressed = false;
    private boolean y2Pressed = false;

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
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder = hardwareMap.get(Servo.class, "feeder");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        //Measure the offsets from the center of the pinpoint device
        //xOffset is the amount in the y-direction from the pinpoint computer that the x-pod is mounted
        //yOffset is the amount in the x-direction from the pinpoint computer that the y-pod is mounted
        odo.setOffsets(4.5,13.2, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        waitForStart();

        intakeTimer.reset();
        feederTimer.reset();
        flywheelTimer.reset();

        while (opModeIsActive()) {

            // ===== FLYWHEEL POWER =====
            /*\
             * B turns off flywheel
             * A sets flywheel to close range power
             * X sets flywheel to medium range power
             * Y sets flywheel to far range power
             * dpad up increases flywheel power by 0.05
             * dpad down decreases flywheel power by 0.05
            \*/
            if (flywheelTimer.milliseconds() > 400) {
                if (gamepad2.b) { // OFF
                    flywheelPower = 0.00;
                } else if (gamepad2.a) { // CLOSE
                    flywheelPower = 0.50;
                } else if (gamepad2.x) { // MEDIUM
                    flywheelPower = 0.75;
                } else if (gamepad2.y) { // FAR
                    flywheelPower = 0.85;
                }

                else if (gamepad2.dpad_up) {
                    flywheelPower += 0.05;
                } else if (gamepad2.dpad_down) {
                    flywheelPower -= 0.05;
                }

                flywheelPower = Math.max(Math.min(flywheelPower, 0.85), 0); // caps flywheel from going above 85% or into reverse
                flywheelTimer.reset();
            }

            // ===== FEEDER FLICKING =====
            /*\
             * right bumper will flick the ball up into the flywheel
            \*/
            if (feederTimer.milliseconds() > 300) {
                if (feederState == FeederState.up) {
                    feeder.setPosition(.22);
                } else if (gamepad2.right_bumper) {
                    feeder.setPosition(.001);
                }
            }

            // ===== INTAKE STATE CHANGING =====
            /*\
             * A toggles between intaking
             * B toggles between rejecting
            \*/
            if (intakeTimer.milliseconds() > 100) {
                if (gamepad1.a) {
                    if (intakeState == IntakeState.on) {
                        intake.setPower(0);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(1);
                        intakeState = IntakeState.on;
                    }
                } else if (gamepad1.b) {
                    if (intakeState == IntakeState.reject) {
                        intake.setPower(0);
                        intakeState = IntakeState.off;
                    } else {
                        intake.setPower(-1);
                        intakeState = IntakeState.reject;
                    }
                }

                intakeTimer.reset();
            }

            //Power the transfer
            if (gamepad1.dpad_up) {
                transfer.setPower(1);
            } else if (gamepad1.dpad_right) {
                transfer.setPower(0);
            } else if (gamepad1.dpad_down) {
                transfer.setPower(-1);
            }

            //Take the heading as measured by the pinpoint device
            //Manipulate the power values so that the robot moves relative to the field not the robot
            angleOffset = odo.getHeading(AngleUnit.DEGREES);
            odo.update();
            axial = (gamepad1.left_stick_y * Math.cos(angleOffset) + gamepad1.left_stick_x * Math.sin(angleOffset));
            lateral = (gamepad1.left_stick_y * Math.sin(angleOffset) - gamepad1.left_stick_x * Math.cos(angleOffset));
            yaw = -gamepad1.right_stick_x;


            leftFrontPower = (axial + lateral + yaw);
            leftBackPower = (axial - lateral + yaw);
            rightFrontPower = (axial - lateral - yaw);
            rightBackPower = (axial + lateral - yaw);

            leftFrontDriveMotor.setPower(leftFrontPower);
            leftBackDriveMotor.setPower(leftBackPower);
            rightFrontDriveMotor.setPower(rightFrontPower);
            rightBackDriveMotor.setPower(rightBackPower);


            telemetry.addData("Power", flywheelPower);
            telemetry.addData("heading", angleOffset);
            telemetry.addData("x", odo.getEncoderX());
            telemetry.addData("y", odo.getEncoderY());
            telemetry.update();
        }
    }

    enum IntakeState {
        off, on, reject
    }

    enum FeederState {
        up, down
    }
}
