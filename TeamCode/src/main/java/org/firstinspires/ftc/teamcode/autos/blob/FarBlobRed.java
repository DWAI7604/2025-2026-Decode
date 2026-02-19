//package org.firstinspires.ftc.teamcode.autos.blob;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.MecanumKinematics;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.PoseStorage;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//import org.firstinspires.ftc.teamcode.autos.actions.Flywheel;
//import org.firstinspires.ftc.teamcode.autos.actions.Intake;
//import org.firstinspires.ftc.teamcode.autos.actions.Turret;
//
//@Autonomous(name="Blob - Far - RED", group="Robot")
//public class FarBlobRed extends LinearOpMode {
//
//    public final MecanumKinematics kinematics = new MecanumKinematics(
//            MecanumDrive.PARAMS.inPerTick * MecanumDrive.PARAMS.trackWidthTicks, MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick);
//
//    @Override
//    public void runOpMode() {
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(62.9, 10.3, Math.toRadians(90)));
//        Flywheel flywheel = new Flywheel(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
//        Turret turret = new Turret(hardwareMap);
//
//        Action moveToLaunch = drive.actionBuilder(new Pose2d(62.9, 10.3, Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(new Vector2d(59, 14), Math.toRadians(45)), 0)
//                .build();
//
//        Action moveThroughIntake1 = drive.actionBuilder(new Pose2d(59, 14, Math.toRadians(45)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(72/2.0, 28), Math.toRadians(90)), Math.toRadians(90))
//                .lineToYConstantHeading(60)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(59, 14), Math.toRadians(45)), 0)
//                .build();
//
//        Action moveThroughIntake2 = drive.actionBuilder(new Pose2d(59, 14, Math.toRadians(45)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/6, 40), Math.toRadians(90)), Math.toRadians(90))
//                .lineToYConstantHeading(60)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(59, 14), Math.toRadians(45)), 0)
//                .build();
//
//        Action moveThroughIntakeHP = drive.actionBuilder(new Pose2d(59, 14, Math.toRadians(45)))
////                .splineToLinearHeading(new Pose2d(new Vector2d(47, 42), Math.toRadians(45)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(new Vector2d(54, 63), Math.toRadians(0)), Math.toRadians(0))
////                .splineTo(new Vector2d(54, 63), Math.toRadians(0))
////                .splineTo(new Vector2d(63, 63), Math.toRadians(0))
//                .lineToXConstantHeading(63)
//                .lineToXConstantHeading(55)
//                .splineToLinearHeading(new Pose2d(new Vector2d(59, 14), Math.toRadians(45)), 0)
//                .build();
//
//        Action moveToPark = drive.actionBuilder(new Pose2d(59, 14, Math.toRadians(45)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(47, 22), Math.toRadians(90)), Math.toRadians(90))
//                .build();
//
//        double delay = 0; // measured in seconds
//        boolean dpadUpPrev = false;
//        boolean dpadDownPrev = false;
//        boolean dpadLeftPrev = false;
//        boolean dpadRightPrev = false;
//
//        while(opModeInInit()) {
//            if (gamepad2.dpad_up != dpadUpPrev) {
//                dpadUpPrev = gamepad2.dpad_up;
//
//                if (dpadUpPrev) {
//                    delay += 1;
//                }
//            } else if (gamepad2.dpad_down != dpadDownPrev) {
//                dpadDownPrev = gamepad2.dpad_down;
//
//                if (dpadDownPrev) {
//                    delay -= 1;
//                }
//            } else if (gamepad2.dpad_right != dpadRightPrev) {
//                dpadRightPrev = gamepad2.dpad_right;
//
//                if (dpadRightPrev) {
//                    delay += 0.25;
//                }
//            } else if (gamepad2.dpad_left != dpadLeftPrev) {
//                dpadLeftPrev = gamepad2.dpad_left;
//
//                if (dpadLeftPrev) {
//                    delay -= 0.25;
//                }
//            }
//
//            telemetry.addData("delay: ", delay);
//            telemetry.update();
//        }
//
//        Action cycle = new SequentialAction(
//                new ParallelAction(
//                        moveToBalls,
//                        intake.intakeIn(4000)
//                )
//        )
//
//        waitForStart();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        new SleepAction(delay),
//
//                        new ParallelAction(
//                                moveToLaunch,
//                                flywheel.velocity(1400),
//                                flywheel.angle(0.00),
//                                flywheel.closeBlocker(),
//                                turret.followGoalRedFar(59, 14, Math.toRadians(49))
//                        ),
//                        flywheel.launchFar(),
//
//                        new ParallelAction(
//                                moveThroughIntake1,
//                                intake.passthrough(),
//                                intake.intakeIn(4600),
//                                flywheel.velocity(1400),
//                                turret.followGoalRedFar(59, 14, Math.toRadians(45))
//                        ),
//                        flywheel.launchFar(),
//
//                        new ParallelAction(
//                                moveThroughIntakeHP,
//                                intake.passthrough(),
//                                intake.intakeIn(4600),
//                                flywheel.velocity(1400),
//                                turret.followGoalRedFar(59, 14, Math.toRadians(42))
//                        ),
//                        flywheel.launchFar(),
//
//                        new ParallelAction(
//                                flywheel.stopAll(),
//                                turret.reset(),
//                                moveToPark
//                        )
//
//                )
//        );
//
//        PoseStorage.currentPose = drive.localizer.getPose();
//    }
//}