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

@Autonomous(name="CloseNineBall - RED", group="Robot")
public class CloseNineBallRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-61, 44, Math.toRadians(-55)));
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action moveToLaunch = drive.actionBuilder(new Pose2d(-61, 44, Math.toRadians(-55)))
                .splineTo(new Vector2d(-46, 24), Math.toRadians(-65))
                .build();

        Action moveThroughIntake1 = drive.actionBuilder(new Pose2d(-46, 24, Math.toRadians(-65)))
                .splineTo(new Vector2d(-70.0/6, 24), Math.toRadians(90))
                .lineToYConstantHeading(51.5)
                .setReversed(true)
                .splineTo(new Vector2d(-46, 24), Math.toRadians(-65 + 180))
                .build();

        Action moveThroughIntake2 = drive.actionBuilder(new Pose2d(-46, 24, Math.toRadians(-65)))
                .splineTo(new Vector2d(70.0/6 + 2, 33), Math.toRadians(90))
                .lineToYConstantHeading(58.5)
                .setReversed(true)
                .splineTo(new Vector2d(-46, 24), Math.toRadians(-65 + 180))
                .build();

        Action moveToPark = drive.actionBuilder(new Pose2d(-46, 24, Math.toRadians(-65)))
                .splineTo(new Vector2d(-44, 14), Math.toRadians(-90))
                .build();

        double delay = 0; // measured in seconds
        boolean dpadUpPrev = false;
        boolean dpadDownPrev = false;

        while(opModeInInit()) {
            if (gamepad2.dpad_up != dpadUpPrev) {
                dpadUpPrev = gamepad2.dpad_up;

                if (dpadUpPrev) {
                    delay += 0.05;
                }
            } else if (gamepad2.dpad_down != dpadDownPrev) {
                dpadDownPrev = gamepad2.dpad_right;

                if (dpadDownPrev) {
                    delay -= 0.05;
                }
            }

            telemetry.addData("delay: ", delay);
            telemetry.update();
        }

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(delay),
                        new ParallelAction(
                                moveToLaunch,
                                flywheel.velocity(1010),
                                flywheel.angle(1),
                                flywheel.prepLaunch()
                        ),
                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(400),
                                intake.intakeIn(400),
                                flywheel.velocity(1010)
                        ),
                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(500),
                                intake.intakeIn(500),
                                flywheel.velocity(1010)
                        ),
                        flywheel.launch(),


                        new ParallelAction(
                                moveThroughIntake1,
                                intake.transferIn(3250),
                                intake.intakeIn(5700),
                                flywheel.velocity(1000)
                        ),


                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(400),
                                intake.intakeIn(400),
                                flywheel.velocity(1000)
                        ),
                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(500),
                                intake.intakeIn(500),
                                flywheel.velocity(1000)
                        ),
                        flywheel.launch(),

                        new ParallelAction(
                                moveThroughIntake2,
                                intake.transferIn(3250),
                                intake.intakeIn(5700),
                                flywheel.velocity(1000)
                        ),


                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(400),
                                intake.intakeIn(400),
                                flywheel.velocity(1000)
                        ),
                        flywheel.launch(),
                        new ParallelAction(
                                intake.transferIn(500),
                                intake.intakeIn(500),
                                flywheel.velocity(1000)
                        ),
                        flywheel.launch(),
                        new ParallelAction(
                                flywheel.velocity(0),
                                moveToPark
                        )
                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();
    }
}
