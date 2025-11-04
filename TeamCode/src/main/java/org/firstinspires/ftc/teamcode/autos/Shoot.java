package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.autos.actions.Flywheel;
import org.firstinspires.ftc.teamcode.autos.actions.Intake;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name="Shoot", group="Robot")
public class Shoot extends LinearOpMode {

    @Override
    public void runOpMode() {
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-39, 53, Math.toRadians(-90)));
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        flywheel.power(0.55, 1.0),
                        flywheel.launch(),
                        flywheel.power(0.0, 1.0),
                        new ParallelAction(
                                intake.transferIn(1000),
                                intake.intakeIn(1000)
                        )
                )
        );

//        PoseStorage.currentPose = drive.localizer.getPose();
    }
}
