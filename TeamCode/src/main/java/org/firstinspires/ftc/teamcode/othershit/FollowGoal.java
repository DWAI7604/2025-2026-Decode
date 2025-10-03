package org.firstinspires.ftc.teamcode.othershit;

import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
//import java.util.List;

import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FollowGoal", group="Linear OpMode")
public class FollowGoal extends LinearOpMode {

    private PinpointLocalizer localizer;
    private Pose2d pose;

    private Limelight3A limelight;
    private LLResult result;

    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;

    private boolean limelightActive = false;
    private boolean limelightValid;

    private State driveState = State.followGoal;

    private ElapsedTime yTimer = new ElapsedTime();

    @Override
    public void runOpMode () {
        localizer = new PinpointLocalizer(hardwareMap, 72.0 / 36653.0, new Pose2d(0, 0, Math.toRadians(0)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
//        limelight.start();

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        yTimer.reset();

        while (opModeIsActive()) {
            telemetry.addData("status", "running");
            localizer.update();
            pose = localizer.getPose();

            if (gamepad1.dpad_down) {
                limelight.start();
            } else if (gamepad1.dpad_up) {
                limelight.stop();
            }

            if (gamepad1.y && yTimer.milliseconds() > 500) {

                if(driveState == State.standard) {
                    driveState = State.followGoal;
                } else if (driveState == State.followGoal){
                    driveState = State.standard;
                }

                yTimer.reset();
            }

            result = limelight.getLatestResult();
            limelightValid = result != null && result.isValid();

            mecanumDrive();

            telemetry.update();
        }
    }

    public double locateGoal () { // returns the yaw value so that the robot turns to the goal

        double yaw = 0;

        if (limelightValid) {

            double tx = result.getTx();

            yaw = tx * 0.02;

            telemetry.addData("Target X", tx);

        } else {

            double robotHeading = pose.heading.toDouble();

            telemetry.addData("robotHeading", robotHeading);

            double xDist = (-55) - pose.position.x;
            double yDist = (55) - pose.position.y;

            telemetry.addData("xDist", xDist);
            telemetry.addData("yDist", yDist);

            double goalHeading = Math.atan(yDist / xDist);

            if (xDist < 0) // because arctan only gives values for x > 0 section of unit circle
                goalHeading = (3.14159) + goalHeading;

            telemetry.addData("goalHeading", goalHeading);

            yaw = (robotHeading - goalHeading) * .50;

        }

        return yaw;

    }

    private void mecanumDrive () {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double heading = pose.heading.toDouble();
        double yaw = 0;

        if (driveState == State.standard)
            yaw = gamepad1.right_stick_x;
        else if (driveState == State.followGoal) {
            yaw = locateGoal();
        }

        // controller
        double axial = (gamepad1.left_stick_y * Math.cos(heading) + gamepad1.left_stick_x * Math.sin(heading));
        double lateral = (gamepad1.left_stick_y * Math.sin(heading) - gamepad1.left_stick_x * Math.cos(heading));

        yaw = Math.min(Math.max(-yaw, -1.0), 1.0);

        telemetry.addData("controller", String.format("%.2f : %.2f : %.2f", axial, lateral, yaw));

        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        
        leftFrontPower  = (axial + lateral + yaw) / denominator;
        leftBackPower   = (axial - lateral + yaw) / denominator;
        rightFrontPower = (axial - lateral - yaw) / denominator;
        rightBackPower  = (axial + lateral - yaw) / denominator;

        telemetry.addData("motors", String.format("%.2f : %.2f : %.2f : %.2f",
                leftFrontPower, leftBackPower, rightFrontPower, rightBackPower));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    enum State {
            standard, followGoal
    }

}
