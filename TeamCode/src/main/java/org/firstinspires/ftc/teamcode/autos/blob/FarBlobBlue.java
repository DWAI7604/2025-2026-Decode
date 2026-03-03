package org.firstinspires.ftc.teamcode.autos.blob;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.autos.actions.Flywheel;
import org.firstinspires.ftc.teamcode.autos.actions.Intake;
import org.firstinspires.ftc.teamcode.autos.actions.Turret;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Autonomous(name="Blob - Far 12 - BLUE", group="Robot")
public class FarBlobBlue extends LinearOpMode {

    private ElapsedTime timer = new ElapsedTime();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            MecanumDrive.PARAMS.inPerTick * MecanumDrive.PARAMS.trackWidthTicks, MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick);

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(62.9, -10.3, Math.toRadians(-90)));
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(8);
        limelight.start();

        Action moveToLaunch = drive.actionBuilder(new Pose2d(62.9, -10.3, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(new Vector2d(59, -14), Math.toRadians(-45)), 0)
                .build();

        Action moveThroughIntake1 = drive.actionBuilder(new Pose2d(59, -14, Math.toRadians(-45)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(72/2.0, -28), Math.toRadians(-90)), Math.toRadians(-90))
                .lineToYConstantHeading(-60)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(59, -14), Math.toRadians(-45)), 0)
                .build();

//        Action moveThroughIntake2 = drive.actionBuilder(new Pose2d(59, 14, Math.toRadians(45)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/6, 40), Math.toRadians(90)), Math.toRadians(90))
//                .lineToYConstantHeading(60)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(59, 14), Math.toRadians(45)), 0)
//                .build();

        Action moveThroughIntakeHP = drive.actionBuilder(new Pose2d(59, -14, Math.toRadians(-45)))
                .splineTo(new Vector2d(44, -45), Math.toRadians(-90))
                .splineTo(new Vector2d(54, -61.5), Math.toRadians(-18))
                .splineTo(new Vector2d(64, -63), Math.toRadians(0))
                .lineToXConstantHeading(55)
                .splineToLinearHeading(new Pose2d(new Vector2d(59, -14), Math.toRadians(-45)), 0)
                .build();

        Action moveToPark = drive.actionBuilder(new Pose2d(59, -14, Math.toRadians(-45)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(47, -22), Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        Action moveThroughBalls;

        Action moveToLimelight = drive.actionBuilder(new Pose2d(59, -14, Math.toRadians(-45)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(60, -12), Math.toRadians(-125)), Math.toRadians(-125))
                .build();

        Action cycle;

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
        timer.reset();

        Actions.runBlocking( // shoot preloads
                new SequentialAction(
                        new SleepAction(delay),

                        new ParallelAction(
                                moveToLaunch,
                                flywheel.velocity(1450),
                                flywheel.angle(0.82),
                                flywheel.closeBlocker(),
                                turret.followGoalBlueFar(59, -14, Math.toRadians(-48))
                        ),
                        new ParallelAction(
                                intake.intakeOut(50),
                                turret.followGoalBlueFar(59, -14, Math.toRadians(-48))
                        ),
                        flywheel.launchFar(),

                        new ParallelAction(
                                moveThroughIntake1,
//                                intake.passthrough(),
                                intake.intakeIn(3000),
                                flywheel.velocity(1450),
                                turret.followGoalBlueFar(59, -14, Math.toRadians(-45))
                        ),
                        new ParallelAction(
                                intake.intakeOut(50),
                                turret.followGoalBlueFar(59, -14, Math.toRadians(-45))
                        ),
                        flywheel.launchFar(),

                        new ParallelAction(
                                moveThroughIntakeHP,
//                                intake.passthrough(),
                                intake.intakeIn(3500),
                                flywheel.velocity(1450),
                                turret.followGoalBlueFar(59, -14, Math.toRadians(-43.5))
                        ),
                        new ParallelAction(
                                intake.intakeOut(50),
                                turret.followGoalBlueFar(59, -14, Math.toRadians(-43.5))
                        ),
                        flywheel.launchFar(),

                        moveToLimelight
                )
        );

//        while (timer.seconds() < 20.0) // run cycle

        double xVal = 60;
        double xCoord, yCoord;

        ArrayList<Double> xCoords = new ArrayList<>();
        ArrayList<Double> yCoords = new ArrayList<>();

        for (int i = 0; i < 11; i++) {
            sleep(20);

            LLResult result = limelight.getLatestResult();
            double[] pythonOutputs = result.getPythonOutput();

            if (pythonOutputs != null && pythonOutputs.length > 0) {
                xCoord = pythonOutputs[6];
                yCoord = pythonOutputs[7];

                xCoords.add(xCoord);
                yCoords.add(yCoord);
            }

        }

        limelight.stop();

        Collections.sort(xCoords);
        Collections.sort(yCoords);

        int size = xCoords.size();
        int middleIndex = size / 2;

        xCoord = xCoords.get(middleIndex);
        yCoord = yCoords.get(middleIndex);

        // SWAPPING FROM RED SIDE TO BLUE SIDE
        xCoord = 640 - xCoord;

        if (size > 3) {
            xVal = LLtoXPos( xCoord, yCoord );

            if (xVal > 50)
                xVal = 64;
        }

        moveThroughBalls = drive.actionBuilder(new Pose2d(47, -22, Math.toRadians(-90)))
                .splineTo(new Vector2d(xVal, -60), Math.toRadians(-90)) // x value = limelight
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(59, -14), Math.toRadians(-45)), 0)
                .build();

        Actions.runBlocking( // cycle
                new SequentialAction(
                        new ParallelAction(
                                moveThroughBalls,
                                intake.intakeIn(2700),
                                flywheel.velocity(1420),
                                turret.followGoalBlueFar(59, -14, Math.toRadians(-45))
                        ),
                        new ParallelAction(
                                intake.intakeOut(200),
                                turret.followGoalBlueFar(59, -14, Math.toRadians(-45))
                        ),
                        flywheel.launchFar()
                )
        );

        Actions.runBlocking( // park
                new ParallelAction(
                        flywheel.stopAll(),
                        turret.reset(),
                        moveToPark
                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();

        // END OF AUTO

//        int index = 0;
//        while (opModeIsActive()) {
//
//            sleep(1000);
//
//            xVal = 35;
//
//            xCoords = new ArrayList<>();
//            yCoords = new ArrayList<>();
//
//            for (int i = 0; i < 11; i++) {
//                sleep(20);
//
//                LLResult result = limelight.getLatestResult();
//                double[] pythonOutputs = result.getPythonOutput();
//
//                if (pythonOutputs != null && pythonOutputs.length > 0) {
//                    xCoord = pythonOutputs[6];
//                    yCoord = pythonOutputs[7];
//
//                    xCoords.add(xCoord);
//                    yCoords.add(yCoord);
//                }
//
//            }
//
//            Collections.sort(xCoords);
//            Collections.sort(yCoords);
//
//            size = xCoords.size();
//            middleIndex = size / 2;
//
//            xCoord = xCoords.get(middleIndex);
//            yCoord = yCoords.get(middleIndex);
//
//            if (size > 3) {
//                xVal = LLtoXPos( xCoord, yCoord );
//            }
//
//            telemetry.addData("limelight x", xCoord);
//            telemetry.addData("limelight y", yCoord);
//            telemetry.addData("position x", xVal);
//            telemetry.addData("index", index);
//            telemetry.update();
//
//            index++;
//        }
    }

    public double LLtoXPos (double limelightX, double limelightY) {
        double x = -0.000000099 * Math.pow(limelightX + limelightY, 3) + 60;

        return Math.max(28, Math.min(x, 66));
    }
}