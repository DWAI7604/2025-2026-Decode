package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        // ====================== INDEPENDENT FAR
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62.9, -10.3, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(new Vector2d(59, -14), Math.toRadians(-45)), 0)
                .waitSeconds(1) // LAUNCH 3 BALLS starting
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(72/2.0, 28), Math.toRadians(90)), Math.toRadians(90))
//                .lineToYConstantHeading(58.5)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(57, 17), Math.toRadians(45)), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS closest strike
//                .splineTo(new Vector2d(44, 45), Math.toRadians(90))
//                .splineTo(new Vector2d(54, 61.5), Math.toRadians(18))
//                .splineTo(new Vector2d(64, 63), Math.toRadians(0))
//                .lineToXConstantHeading(55)
//                .splineToLinearHeading(new Pose2d(new Vector2d(55, 15), Math.toRadians(45)), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS human player
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(45, 14), Math.toRadians(160)), Math.toRadians(160))
//                .splineTo(new Vector2d(72.0/6, 40), Math.toRadians(90))
//                .lineToYConstantHeading(58.5)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(59, 14), Math.toRadians(45)), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS middle strike
                        .splineTo(new Vector2d(61, -30), Math.toRadians(-90))
                .splineTo(new Vector2d(63, -60), Math.toRadians(-90)) // x value = limelight
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(59, -14), Math.toRadians(-45)), 0)
                .waitSeconds(1) // LAUNCH FROM INTAKE
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(59, -14), Math.toRadians(-90)), Math.toRadians(-90))
                .build());

        // ====================== PUSH FAR
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(72 - 17.0/2.0, 3 + 14 / 2.0, Math.toRadians(-90)))
//                .setTangent(Math.toRadians(-90))
//                .lineToY((3 + 14 / 2.0) + 30)
//                .setReversed(true)
//                .lineToY(3 + 14 / 2.0)
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/2, 28), Math.toRadians(90)), Math.toRadians(90))
//                .lineToYConstantHeading(58.5)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(57, 17), Math.toRadians(45)), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS closest strike
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/6, 40), Math.toRadians(90)), Math.toRadians(90))
//                .lineToYConstantHeading(58.5)
//                .setReversed(true)
//                .splineTo(new Vector2d(-40, 17), Math.toRadians(180))
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .splineTo(new Vector2d(-70.0/6, 28), Math.toRadians(90))
//                .lineToYConstantHeading(52)
//                .setReversed(true)
//                .splineTo(new Vector2d(-40, 17), Math.toRadians(180))
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .build());

//        // ====================== GATE CLOSE
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61, 44, Math.toRadians(-55)))
//                .splineToLinearHeading(new Pose2d(-40, 17, Math.toRadians(0)), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .splineTo(new Vector2d(70.0/6, 33), Math.toRadians(90))
//                .lineToYConstantHeading(58.5)
//                .lineToYConstantHeading(42)
//                .splineToLinearHeading(new Pose2d(-70.0/6 + 16, 52, Math.toRadians(0)), Math.toRadians(90)) // HIT GATE
//                .lineToYConstantHeading(26)
//                .setReversed(true)
//                .splineTo(new Vector2d(-40, 17), Math.toRadians(180))
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .build());


        // ====================== INDEPENDENT CLOSE
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61, 44, Math.toRadians(-55)))
//                .splineToLinearHeading(new Pose2d(-40, 17, Math.toRadians(0)), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .splineTo(new Vector2d(-70.0/6, 28), Math.toRadians(90))
//                .lineToYConstantHeading(52)
//                .setReversed(true)
//                .splineTo(new Vector2d(-40, 17), Math.toRadians(180))
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .splineTo(new Vector2d(70.0/6, 33), Math.toRadians(90))
//                .lineToYConstantHeading(58.5)
//                .setReversed(true)
//                .splineTo(new Vector2d(-40, 17), Math.toRadians(180))
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .splineTo(new Vector2d(70.0/6 + 24, 39), Math.toRadians(90))
//                .lineToYConstantHeading(58.5)
//                .setReversed(true)
//                .splineTo(new Vector2d(-40, 17), Math.toRadians(180))
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}