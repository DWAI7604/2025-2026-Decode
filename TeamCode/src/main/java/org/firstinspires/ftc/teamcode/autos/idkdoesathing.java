package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseStorage;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name="idkdoesathing", group="Robot")
public class idkdoesathing extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-50, 50, Math.toRadians(-55)));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-50, 50, Math.toRadians(-55)))
                    .waitSeconds(1)
                    .lineToY(30)
                    .waitSeconds(1)
                    .splineTo(new Vector2d(-12, 30), Math.toRadians(90))
                        .waitSeconds(1)
                    .lineToY(50)
                        .waitSeconds(1)
                    .setReversed(true)
                    .splineTo(new Vector2d(-35, 30), Math.toRadians(-55 + 180))
                        .waitSeconds(1)
                    .build());

        PoseStorage.currentPose = drive.localizer.getPose();
    }
}
