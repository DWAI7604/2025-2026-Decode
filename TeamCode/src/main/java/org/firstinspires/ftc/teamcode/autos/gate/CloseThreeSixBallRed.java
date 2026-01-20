//package org.firstinspires.ftc.teamcode.autos.gate;
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
//
//@Autonomous(name="Gate - Close 3+6 - RED", group="Robot")
//public class CloseThreeSixBallRed extends LinearOpMode {
//
//    public final MecanumKinematics kinematics = new MecanumKinematics(
//            MecanumDrive.PARAMS.inPerTick * MecanumDrive.PARAMS.trackWidthTicks, MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick);
//
//    @Override
//    public void runOpMode() {
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-61, 44, Math.toRadians(-55)));
//        Flywheel flywheel = new Flywheel(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
//
//        Action moveToLaunch = drive.actionBuilder(new Pose2d(-61, 44, Math.toRadians(-55)))
//                .splineTo(new Vector2d(-46, 24), Math.toRadians(-59))
//                .build();
//
//        Action moveThroughIntake1 = drive.actionBuilder(new Pose2d(-46, 24, Math.toRadians(-59)))
//                .splineTo(new Vector2d(-70.0/6 + 3, 24), Math.toRadians(90))
//                .lineToYConstantHeading(52,  kinematics.new WheelVelConstraint(55))
//                .lineToYConstantHeading(46)
//                .splineToLinearHeading(new Pose2d(-70.0/6 + 7, 52, Math.toRadians(180)), Math.toRadians(90)) // HIT GATE
//                .lineToYConstantHeading(57)
//                .lineToYConstantHeading(50)
//                .splineToLinearHeading(new Pose2d(-46, 24, Math.toRadians(-59)), 0)
//                .build();
//
//        Action moveThroughIntake2 = drive.actionBuilder(new Pose2d(-46, 24, Math.toRadians(-59)))
//                .splineTo(new Vector2d(70.0/6 + 4, 33), Math.toRadians(90))
//                .lineToYConstantHeading(58.5,  kinematics.new WheelVelConstraint(55))
//                .setReversed(true)
//                .splineTo(new Vector2d(-46, 24), Math.toRadians(-59 + 180))
//                .build();
//
//        Action moveToPark = drive.actionBuilder(new Pose2d(-46, 24, Math.toRadians(-59)))
//                .splineTo(new Vector2d(-44, 14), Math.toRadians(-90))
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
//        waitForStart();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        new SleepAction(delay),
//
//
//                        new ParallelAction(
//                                moveToLaunch,
//                                flywheel.velocity(980),
//                                flywheel.angle(1),
//                                flywheel.closeBlocker()
//                        ),
//                        flywheel.launch(),
//
//
//                        new ParallelAction(
//                                moveThroughIntake1,
//                                intake.passthrough(),
//                                intake.intakeIn(4600),
//                                flywheel.velocity(980)
//                        ),
//                        flywheel.launch(),
//
//
//                        new ParallelAction(
//                                moveThroughIntake2,
//                                intake.passthrough(),
//                                intake.intakeIn(4600),
//                                flywheel.velocity(1000)
//                        ),
//                        flywheel.launch(),
//
//
//                        new ParallelAction(
//                                flywheel.stopAll(),
//                                moveToPark
//                        )
//                )
//        );
//
//        PoseStorage.currentPose = drive.localizer.getPose();
//    }
//}