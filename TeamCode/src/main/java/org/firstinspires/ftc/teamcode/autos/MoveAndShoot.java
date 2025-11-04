package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.autos.actions.Flywheel;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name="MoveAndShoot", group="Robot")
public class MoveAndShoot extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-39, 53, Math.toRadians(-90)));
        Flywheel flywheel = new Flywheel(hardwareMap);

        Action toLaunch = drive.actionBuilder(new Pose2d(-50, 50, Math.toRadians(-55)))
                .waitSeconds(1)
                .splineTo(new Vector2d(-35, 30), Math.toRadians(-55)) // firing location
                .waitSeconds(1)
                .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        toLaunch,
                        flywheel.power(0.6, 0.3),
                        flywheel.launch()
                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();
    }
}