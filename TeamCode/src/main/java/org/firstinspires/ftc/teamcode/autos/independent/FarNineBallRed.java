package org.firstinspires.ftc.teamcode.autos.independent;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MecanumKinematics;
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
import org.firstinspires.ftc.teamcode.autos.actions.Turret;

@Autonomous(name="Indep - Far 3(9) - RED", group="Robot")
public class FarNineBallRed extends LinearOpMode {

    public final MecanumKinematics kinematics = new MecanumKinematics(
            MecanumDrive.PARAMS.inPerTick * MecanumDrive.PARAMS.trackWidthTicks, MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick);

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(72 - 17.0/2.0, 3 + 14 / 2.0, Math.toRadians(90)));
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        Action moveToLaunch = drive.actionBuilder(new Pose2d(72 - 17.0/2.0, 3 + 14 / 2.0, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(new Vector2d(57, 17), Math.toRadians(45)), 0)
                .build();

        Action moveThroughIntake1 = drive.actionBuilder(new Pose2d(72 - 17.0/2.0, 3 + 14 / 2.0, 0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(72/2.0, 28), Math.toRadians(90)), Math.toRadians(90))
                .lineToYConstantHeading(58.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(57, 17), Math.toRadians(45)), 0)
                .build();

        Action moveThroughIntake2 = drive.actionBuilder(new Pose2d(57, 17, Math.toRadians(45)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/6, 40), Math.toRadians(90)), Math.toRadians(90))
                .lineToYConstantHeading(58.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(57, 17), Math.toRadians(45)), 0)
                .build();

        Action moveThroughIntakeHP = drive.actionBuilder(new Pose2d(57, 17, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(new Vector2d(53, 45), Math.toRadians(90)), Math.toRadians(90))
                .splineTo(new Vector2d(60, 60), Math.toRadians(0))
                .lineToXConstantHeading(55)
                .splineToLinearHeading(new Pose2d(new Vector2d(57, 17), Math.toRadians(45)), 0)
                .build();

        Action moveToPark = drive.actionBuilder(new Pose2d(57, 17, Math.toRadians(45)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(72 - 14 / 2.0, 38), Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0.001)
                .lineToYConstantHeading(45) // park
                .build();

        double delay = 0; // measured in seconds
        boolean dpadUpPrev = false;
        boolean dpadDownPrev = false;
        boolean dpadLeftPrev = false;
        boolean dpadRightPrev = false;

        while(opModeInInit()) {
            if (gamepad2.dpad_up != dpadUpPrev) {
                dpadUpPrev = gamepad2.dpad_up;

                if (dpadUpPrev) {
                    delay += 1;
                }
            } else if (gamepad2.dpad_down != dpadDownPrev) {
                dpadDownPrev = gamepad2.dpad_down;

                if (dpadDownPrev) {
                    delay -= 1;
                }
            } else if (gamepad2.dpad_right != dpadRightPrev) {
                dpadRightPrev = gamepad2.dpad_right;

                if (dpadRightPrev) {
                    delay += 0.25;
                }
            } else if (gamepad2.dpad_left != dpadLeftPrev) {
                dpadLeftPrev = gamepad2.dpad_left;

                if (dpadLeftPrev) {
                    delay -= 0.25;
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
                                flywheel.velocity(2140),
                                flywheel.angle(0.35),
                                flywheel.closeBlocker(),
                                turret.followGoalRed(72 - 17.0/2.0, 3 + 14 / 2.0, 0)
                        ),
                        flywheel.launchFar(),

//                        new ParallelAction(
//                                moveThroughIntake1,
//                                intake.passthrough(),
//                                intake.intakeIn(4600),
//                                flywheel.velocity(2140),
//                                turret.followGoalRed(57, 17, Math.toRadians(45))
//                        ),
//                        flywheel.launchFar(),
//
//                        new ParallelAction(
//                                moveThroughIntakeHP,
//                                intake.passthrough(),
//                                intake.intakeIn(4600),
//                                flywheel.velocity(2140)
////                                turret.followGoalRed(-40, 17, Math.toRadians(0))
//                        ),
//                        flywheel.launchFar(),
//
//                        new ParallelAction(
//                                moveThroughIntake2,
//                                intake.passthrough(),
//                                intake.intakeIn(4600),
//                                flywheel.velocity(2140)
////                                turret.followGoalRed(-40, 17, Math.toRadians(0))
//                        ),
//                        flywheel.launchFar(),

                        new ParallelAction(
                                flywheel.stopAll(),
                                turret.reset(),
                                moveToPark
                        )

                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();
    }
}