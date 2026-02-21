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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.autos.actions.Flywheel;
import org.firstinspires.ftc.teamcode.autos.actions.Intake;
import org.firstinspires.ftc.teamcode.autos.actions.Turret;

@Autonomous(name="Indep - Close 9 - RED", group="Robot")
public class CloseNineBallRed extends LinearOpMode {

    public final MecanumKinematics kinematics = new MecanumKinematics(
            MecanumDrive.PARAMS.inPerTick * MecanumDrive.PARAMS.trackWidthTicks, MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick);

    @Override
    public void runOpMode() {

//        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
//        turretMotor.setDirection(DcMotor.Direction.REVERSE);
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
//        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-58.4, 45.0, -0.9427));
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        Action moveToLaunch = drive.actionBuilder(new Pose2d(-58.4, 45.0, -0.9427))
                .splineToLinearHeading(new Pose2d(-40, 17, Math.toRadians(0)), 0)
                .build();

        Action moveThroughIntake1 = drive.actionBuilder(new Pose2d(-40, 17, Math.toRadians(0)))
                .splineTo(new Vector2d(-70.0/6, 28), Math.toRadians(90))
                .lineToYConstantHeading(52,  kinematics.new WheelVelConstraint(55))
                .setReversed(true)
                .splineTo(new Vector2d(-40, 17), Math.toRadians(180))
                .build();

        Action moveThroughIntake2 = drive.actionBuilder(new Pose2d(-40, 17, Math.toRadians(0)))
                .splineTo(new Vector2d(70.0/6, 33), Math.toRadians(90))
                .lineToYConstantHeading(58.5,  kinematics.new WheelVelConstraint(55))
                .setReversed(true)
                .splineTo(new Vector2d(-40, 17), Math.toRadians(180))
                .build();

        Action moveThroughIntake3 = drive.actionBuilder(new Pose2d(-40, 17, Math.toRadians(0)))
                .splineTo(new Vector2d(70.0/6 + 24, 39), Math.toRadians(90))
                .lineToYConstantHeading(58.5,  kinematics.new WheelVelConstraint(55))
                .setReversed(true)
                .splineTo(new Vector2d(-40, 17), Math.toRadians(180))
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
                                flywheel.velocity(1020),
                                flywheel.angle(0.52),
                                flywheel.closeBlocker(),
                                turret.followGoalRed(-40, 17, Math.toRadians(2))
                        ),
                        flywheel.launchClose(),

                        new ParallelAction(
                                moveThroughIntake1,
//                                intake.passthrough(),
                                intake.intakeIn(2500),
                                flywheel.velocity(1020),
                                turret.followGoalRed(-40, 17, Math.toRadians(4))
                        ),
                        flywheel.launchClose(),

                        new ParallelAction(
                                moveThroughIntake2,
//                                intake.passthrough(),
                                intake.intakeIn(2800),
                                flywheel.velocity(1020)
                        ),
                        flywheel.launchClose(),

//                        new ParallelAction(
//                                moveThroughIntake3,
////                                intake.passthrough(),
//                                intake.intakeIn(2900),
//                                flywheel.velocity(1020),
//                                turret.followGoalRed(-40, 17, Math.toRadians(6))
//                        ),
//                        flywheel.launchClose(),

                        new ParallelAction(
                                flywheel.stopAll(),
                                turret.reset()
                        )

                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();
    }
}