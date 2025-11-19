package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.autos.actions.Flywheel;
import org.firstinspires.ftc.teamcode.autos.actions.Intake;

@Autonomous(name="FarSixBall - RED", group="Robot")
public class FarSixBallRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(57, 12, -0.38));
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action moveToLaunch = drive.actionBuilder(new Pose2d(60, 16, 0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(55, 18), -0.38), 0)
                .build();

        Action moveThroughIntake = drive.actionBuilder(new Pose2d(55, 18, -0.38))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/2, 28), Math.toRadians(90)), Math.toRadians(90))
                .lineToYConstantHeading(60)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(55, 18), -0.38), 0)
                .build();

        Action moveToPark = drive.actionBuilder(new Pose2d(55, 18, -0.38))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(60, 38), Math.toRadians(-90)), 0)
                .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        moveToLaunch,
                        moveThroughIntake,
                        moveToPark
                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();
    }
}