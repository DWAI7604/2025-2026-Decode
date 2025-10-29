package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="RedLongPath", group="Robot")
public class RedLongPath extends LinearOpMode {

    private Limelight3A limelight;
    private Pattern pattern = Pattern.Unknown;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-39, 53, Math.toRadians(-90)));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-50, 50, Math.toRadians(-55)))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(-35, 30), Math.toRadians(-55)) // firing location
                        .waitSeconds(1)
                        // closest line of artifacts
                        .splineTo(new Vector2d(-70.0/6, 28), Math.toRadians(90))
                        .lineToY(49)
                        .setReversed(true)
                        .splineTo(new Vector2d(-35, 30), Math.toRadians(180 - 55)) // firing location
                        .waitSeconds(1)
                        .setReversed(false)
                        // second closest line of artifacts
                        .splineTo(new Vector2d(70.0/6, 30), Math.toRadians(90))
                        .lineToY(49)
                        .setReversed(true)
                        .splineTo(new Vector2d(-35, 30), Math.toRadians(180 - 55)) // firing location
                        .waitSeconds(1)
                        .setReversed(false)
                        // third closest line of artifacts
                        .splineTo(new Vector2d(70.0/2, 34), Math.toRadians(90))
                        .lineToY(49)
                        .setReversed(true)
                        .splineTo(new Vector2d(-35, 30), Math.toRadians(180 - 55)) // firing location
                        .waitSeconds(1)
                        .setReversed(false)
                        // parking
                        .splineTo(new Vector2d(-35, 12), Math.toRadians(-90))

                        .build());

        PoseStorage.currentPose = drive.localizer.getPose();
    }

    public enum Pattern {
        Unknown, GPP, PGP, PPG
    }
}
