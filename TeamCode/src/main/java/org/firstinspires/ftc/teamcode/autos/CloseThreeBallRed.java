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

@Autonomous(name="CloseThreeBall - RED", group="Robot")
public class CloseThreeBallRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-57, 44, Math.toRadians(-55)));
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action moveToLaunch = drive.actionBuilder(new Pose2d(-57, 44, Math.toRadians(-55)))
                .splineTo(new Vector2d(-42, 30), Math.toRadians(-58))
                .build();

        Action moveToPark = drive.actionBuilder(new Pose2d(-42, 30, Math.toRadians(-58)))
                .splineTo(new Vector2d(-35, 12), Math.toRadians(-90))
                .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                moveToLaunch,
                                flywheel.power(0.6, 3.0),
                                flywheel.angle(1.0)
                        ),
                        flywheel.power(0.525, 2.0),
                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(1000),
                                intake.intakeIn(1000)
                        ),
                        flywheel.launch(),
                        new SleepAction(1),
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
