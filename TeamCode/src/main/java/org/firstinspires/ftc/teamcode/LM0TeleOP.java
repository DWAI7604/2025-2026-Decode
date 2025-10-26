package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class LM0TeleOP extends RobotLinearOpMode {

    private DcMotor leftFrontDriveMotor;
    private DcMotor leftBackDriveMotor;
    private DcMotor rightFrontDriveMotor;
    private DcMotor rightBackDriveMotor;
    private DcMotor flywheel;
    private DcMotor intake;
    private Servo feeder;
    private GoBildaPinpointDriver odo;

    private CRServo transfer;

    private double flywheelPower;
    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;
    private double axial;
    private double lateral;
    private double yaw;
    private double angleOffset;
    private boolean a1Pressed;
    private boolean b1Pressed;
    private boolean y1Pressed;
    private boolean x1Pressed;
    private boolean a2Pressed;
    private boolean b2Pressed;
    private boolean y2Pressed;
    private boolean x2Pressed;

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



        //Establish starting values for booleans
        y2Pressed = false;
        a1Pressed = false;
        b1Pressed = false;


        waitForStart();
        while (opModeIsActive()) {
            
            //Power the flywheel so that:
            // - Pressing Y incrementally increases the power by 0.05
            // - Pressing A incrementally decreases the power by 0.05
            // - Pressing B sets the power to 0 and resets the booleans
            // - If the power goes low enough the flywheel shuts down
            if (gamepad2.y && !y2Pressed) {
                flywheelPower = .5;
                y2Pressed = true;
                sleep(400);
            } else if (gamepad2.y && y2Pressed) {
                flywheelPower += 0.05;
                sleep(400);
            } else if (gamepad2.a && y2Pressed) {
                flywheelPower -= 0.05;
                sleep(400);
            } else if (gamepad2.b) {
                flywheelPower = 0;
                y2Pressed = false;
            } else if (flywheelPower < 0.3) {
                flywheelPower = 0;
            }
            
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel.setPower(flywheelPower);

            //Pressing the right bumper will launch the ball up and return the feeder to original position
            if (gamepad2.right_bumper) {

                feeder.setPosition(.001);
                sleep(300);
                feeder.setPosition(.22);
            }

            //If gamepad1.a is pressed then the intake will intake, pressing it again will power it off
            if (gamepad1.a && !a1Pressed) {
                intake.setPower(1);
                a1Pressed = true;
                sleep(400);
            } else if (gamepad1.a && a1Pressed) {
                intake.setPower(0);
                a1Pressed = false;
                sleep(400);
            }

            //If gamepad1.b is pressed the intake will spit out, pressing it again will power it off
            if (gamepad1.b && !b1Pressed) {
                intake.setPower(-1);
                b1Pressed = true;
            } else if (gamepad1.b && b1Pressed) {
                intake.setPower(0);
                b1Pressed = false;
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
}
