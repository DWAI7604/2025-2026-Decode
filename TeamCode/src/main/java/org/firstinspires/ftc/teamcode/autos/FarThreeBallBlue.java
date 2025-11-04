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

@Autonomous(name="FarThreeBall - BLUE", group="Robot")
public class FarThreeBallBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(57, -12, 0.38));
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action moveToPark = drive.actionBuilder(new Pose2d(57, -12, 0.38))
                .setReversed(true)
                .splineTo(new Vector2d(55, -38), Math.toRadians(90))
                .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                flywheel.power(0.8, 4.0),
                                flywheel.angle(0.15)
                        ),
                        flywheel.power(0.675, 4.0),
                        flywheel.launch(),
                        new ParallelAction(
                                flywheel.power(0.675, 0.0),
                                intake.transferIn(1000),
                                intake.intakeIn(1000)
                        ),
                        new SleepAction(2),
                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(1000),
                                intake.intakeIn(1000)
                        ),
                        new SleepAction(2),
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