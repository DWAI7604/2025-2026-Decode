package org.firstinspires.ftc.teamcode.othershit;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp
public class headlessDrive extends LinearOpMode{

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDriveMotor;
    private DcMotor rightFrontDriveMotor;
    private DcMotor leftBackDriveMotor;
    private DcMotor rightBackDriveMotor;
    double angleOffset;
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    @Override
    public void runOpMode() {

        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFront");

        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");


        odo.setOffsets(0, 0, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            double axial;
            double lateral;
            double yaw;
            angleOffset = odo.getHeading(AngleUnit.RADIANS);

            axial = (gamepad1.left_stick_y * Math.cos(angleOffset) + gamepad1.left_stick_x * Math.sin(angleOffset));
            lateral = (gamepad1.left_stick_y * Math.sin(angleOffset) - gamepad1.left_stick_x * Math.cos(angleOffset));
            yaw = -gamepad1.right_stick_x;




            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
//            leftBackPower = ((axial * Math.cos(angleOffset)) - (lateral * Math.sin(angleOffset +90 )) + yaw);
//            rightBackPower = ((axial * Math.cos(angleOffset)) + (lateral * Math.sin(angleOffset +90 )) - yaw);
//            leftFrontPower = ((axial * Math.cos(angleOffset)) + (lateral * Math.sin(angleOffset +90 )) + yaw );
//            rightFrontPower = ((axial * Math.cos(angleOffset)) - (lateral * Math.sin(angleOffset +90 )) - yaw);

            leftBackDriveMotor.setPower(leftBackPower);
            leftFrontDriveMotor.setPower(leftFrontPower);
            rightBackDriveMotor.setPower(rightBackPower);
            rightFrontDriveMotor.setPower(rightFrontPower);

            telemetry.addData("leftFront", leftFrontPower);
            telemetry.addData("leftBack", leftBackPower);
            telemetry.addData("rightFront", rightFrontPower);
            telemetry.addData("rightBack", rightBackPower);
            telemetry.addData("angle", angleOffset);
            telemetry.update();
            odo.update();


        }




    }
}