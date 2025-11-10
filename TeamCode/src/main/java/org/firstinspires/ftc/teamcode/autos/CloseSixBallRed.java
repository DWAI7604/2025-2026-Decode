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

@Autonomous(name="CloseSixBall - RED", group="Robot")
public class CloseSixBallRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-57, 44, Math.toRadians(-55)));
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action moveToLaunch = drive.actionBuilder(new Pose2d(-57, 44, Math.toRadians(-55)))
                .splineToLinearHeading(new Pose2d(new Vector2d(-24, 24), Math.toRadians(-55)), 0)
                .build();

        Action moveThroughIntake = drive.actionBuilder(new Pose2d(-24, 24, Math.toRadians(-55)))
                .splineToSplineHeading(new Pose2d(new Vector2d(-70.0/6, 28), Math.toRadians(90)), Math.toRadians(90))
                .lineToYConstantHeading(52)
                .setReversed(true)
                .splineTo(new Vector2d(-24, 24), Math.toRadians(-55 + 180))
                .build();

        Action moveToPark = drive.actionBuilder(new Pose2d(-24, 24, Math.toRadians(-55 + 180)))
                .splineToLinearHeading(new Pose2d(new Vector2d(-35, 12), Math.toRadians(-90)), 0)
                .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                moveToLaunch,
                                flywheel.power(0.60, 3.0),
                                flywheel.angle(1.0)
                        ),
                        flywheel.power(0.5, 2.0),
                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(1000),
                                intake.intakeIn(1000)
                        ),
                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(1000),
                                intake.intakeIn(1000)
                        ),
                        flywheel.launch(),
                        new ParallelAction(
                                moveThroughIntake,
                                intake.transferIn(3500),
                                intake.intakeIn(3500)
                        ),
                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(1000),
                                intake.intakeIn(1000)
                        ),
                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(1000),
                                intake.intakeIn(1000)
                        ),
                        flywheel.launch(),
                        new ParallelAction(
                                flywheel.power(0.0, 0),
                                moveToPark
                        )
                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();
    }
}
