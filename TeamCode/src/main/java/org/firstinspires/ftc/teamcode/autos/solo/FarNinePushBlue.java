package org.firstinspires.ftc.teamcode.autos.solo;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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

@Autonomous(name="Push - Far 9 - BLUE", group="Robot")
public class FarNinePushBlue extends LinearOpMode {

    public final MecanumKinematics kinematics = new MecanumKinematics(
            MecanumDrive.PARAMS.inPerTick * MecanumDrive.PARAMS.trackWidthTicks, MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick);

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(63.5, -15 / 2.0 - 4, Math.toRadians(90)));
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action moveToLaunch = drive.actionBuilder(new Pose2d(63.5, -15 / 2.0 - 4, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .lineToY(-35,  kinematics.new WheelVelConstraint(30), new ProfileAccelConstraint(-20, 30))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(57, -17), 0.38 + Math.toRadians(-2) ), 0)
                .build();

        Action moveThroughIntake1 = drive.actionBuilder(new Pose2d(57, -17, 0.38 + Math.toRadians(-2) ))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/2 + 4 - 12, -28), Math.toRadians(-90)), Math.toRadians(-90))
                .lineToYConstantHeading(-63)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(57, -17), 0.38), 0)
                .build();

        Action moveThroughIntake2 = drive.actionBuilder(new Pose2d(57, -17, 0.38))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/6 + 6 - 12, -28), Math.toRadians(-90)), Math.toRadians(-90))
                .lineToYConstantHeading(-63)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(57, -17), 0.38 - Math.toRadians(-1) ), 0)
                .build();

        Action moveToPark = drive.actionBuilder(new Pose2d(57, -17, 0.38 - Math.toRadians(-1) ))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(64.5 - 3, -38), Math.toRadians(-90)), 0)
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
                                flywheel.velocity(1480),
                                flywheel.angle(0),
                                flywheel.closeBlocker()
                        ),
                        flywheel.launch(),


                        new ParallelAction(
                                moveThroughIntake1,
                                intake.passthrough(),
                                intake.intakeIn(5000),
                                flywheel.velocity(1480)
                        ),
                        flywheel.launch(),


                        new ParallelAction(
                                moveThroughIntake2,
                                intake.passthrough(),
                                intake.intakeIn(5000),
                                flywheel.velocity(1480)
                        ),
                        flywheel.launch(),


                        new ParallelAction(
                                flywheel.stopAll(),
                                moveToPark
                        )
                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();
    }
}