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
    private Servo angleAdjustServo;
    private CRServo transfer; // this is that rubber band thingy
    private Servo flipServo;

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
    private double angleAdjust;
    private IntakeState intakeState = IntakeState.off;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private FeederState feederState = FeederState.down;
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime flywheelTimer = new ElapsedTime();
    private ElapsedTime angleTimer = new ElapsedTime();
    private ElapsedTime flipTimer = new ElapsedTime();
    private FlipState flipState = FlipState.up;
    private boolean a1Prev = false;
    private boolean b1Prev = false;
    private boolean a2Prev = false;
    private boolean b2Prev = false;

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
        angleAdjustServo = hardwareMap.get(Servo.class, "angleAdjust");
        flipServo = hardwareMap.get(Servo.class, "flap");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        //Measure the offsets from the center of the pinpoint device
        //xOffset is the amount in the y-direction from the pinpoint computer that the x-pod is mounted
        //yOffset is the amount in the x-direction from the pinpoint computer that the y-pod is mounted
        odo.setOffsets(4.5,13.2, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        waitForStart();

        intakeTimer.reset();
        feederTimer.reset();
        flywheelTimer.reset();
        angleTimer.reset();
        flipTimer.reset();

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
                    flywheelPower = 0.60;
                } else if (gamepad2.y) { // FAR
                    flywheelPower = 0.70;
                }

                else if (gamepad2.dpad_up) {
                    flywheelPower += 0.05;
                } else if (gamepad2.dpad_down) {
                    flywheelPower -= 0.05;
                }

                flywheelPower = Math.max(Math.min(flywheelPower, 0.85), 0); // caps flywheel from going above 85% or into reverse
                flywheel.setPower(flywheelPower);
                flywheelTimer.reset();
            }

            // ===== FEEDER FLICKING =====
            /*\
             * right bumper will flick the ball up into the flywheel
            \*/
            if (feederTimer.milliseconds() > 300) {
                if (feederState == FeederState.up) {
                    feeder.setPosition(.22);
                    feederState = FeederState.down;
                } else if (gamepad2.right_bumper) {
                    feeder.setPosition(.001);
                    feederState = FeederState.up;
                }

                feederTimer.reset();
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
                        intake.setPower(1);
                        intakeState = IntakeState.on;
                    }
                }

            } else if (b1Prev != gamepad1.b) {

                b1Prev = gamepad1.b;

                if (b1Prev) {
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
            if (angleTimer.milliseconds() > 100) {
                if (gamepad2.dpad_left) {
                    angleAdjust += 0.05;
                    angleTimer.reset();
                } else if (gamepad2.dpad_right) {
                    angleAdjust -= 0.05;
                    angleTimer.reset();
                }

                angleAdjust = Math.max(Math.min(angleAdjust, 1.0), 0);

                angleAdjustServo.setPosition(angleAdjust);
            }

            // ===== FLIP THAT INTERNAL FLAP =====
            /*\
             * left bumper toggles it
            \*/
            if (flipTimer.milliseconds() > 200 && gamepad2.left_bumper) {
                if (flipState == FlipState.down) { // move up
                    flipState = FlipState.up;
                    flipServo.setPosition(0.0);
                } else {
                    flipState = FlipState.down;
                    flipServo.setPosition(0.1);
                }

                flipTimer.reset();
            }

            //Take the heading as measured by the pinpoint device
            //Manipulate the power values so that the robot moves relative to the field not the robot
            angleOffset = odo.getHeading(AngleUnit.RADIANS);
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
            telemetry.addData("angle of flywheel", angleAdjust);
            telemetry.addData("flipServo: ", flipState);
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

    enum FlipState {
        up, down
    }
}
