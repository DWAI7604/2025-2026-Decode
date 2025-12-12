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
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 14)
                .build();

        // ====================== INDEPENDENT FAR
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(63.5, 15 / 2.0, 0))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(57, 17), -0.38 ), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/2, 28), Math.toRadians(90)), Math.toRadians(90))
//                .lineToYConstantHeading(63)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(57, 17), -0.38), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS
////                .setReversed(true)
////                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/6, 28), Math.toRadians(90)), Math.toRadians(90))
////                .lineToYConstantHeading(60)
////                .setReversed(true)
////                .splineToLinearHeading(new Pose2d(new Vector2d(55, 18), -0.38), 0)
////                .waitSeconds(1) // LAUNCH 3 BALLS
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(64.5, 38), Math.toRadians(90)), Math.toRadians(90))
//                        .waitSeconds(0.001)
//                        .lineToYConstantHeading(63)
//                .build());

        // ====================== SOLO FAR
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(63.5, 15 / 2.0, Math.toRadians(-90)))
//                .setTangent(Math.toRadians(-90))
//                .lineToY(7.5 + 30)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(57, 17), -0.38 ), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/2, 28), Math.toRadians(90)), Math.toRadians(90))
//                .lineToYConstantHeading(63)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(57, 17), -0.38), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS
////                .setReversed(true)
////                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/6, 28), Math.toRadians(90)), Math.toRadians(90))
////                .lineToYConstantHeading(60)
////                .setReversed(true)
////                .splineToLinearHeading(new Pose2d(new Vector2d(55, 18), -0.38), 0)
////                .waitSeconds(1) // LAUNCH 3 BALLS
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(64.5, 38), Math.toRadians(90)), 0)
//                .build());

        // ====================== INDEPENDENT CLOSE
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61, 44, Math.toRadians(-55)))
                .splineTo(new Vector2d(-46, 24), Math.toRadians(-59))
                .waitSeconds(1) // LAUNCH 3 BALLS
                .splineTo(new Vector2d(-70.0/6 + 3, 24), Math.toRadians(90))
                .lineToYConstantHeading(52)
                        .lineToYConstantHeading(46)
                .splineToLinearHeading(new Pose2d(-70.0/6 + 8, 52, Math.toRadians(180)), Math.toRadians(90)) // HIT GATE
                                .lineToYConstantHeading(57)
                                .lineToYConstantHeading(50)
//                .splineToLinearHeading(new Pose2d((-70.0/6 + 3) + 3, 52 + 5, Math.toRadians(180)), 0) // HIT GATE
                                .splineToLinearHeading(new Pose2d(-46, 24, Math.toRadians(-59)), 0)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-46, 24, Math.toRadians((-59))), 0)
                .waitSeconds(1) // LAUNCH 3 BALLS
//                .splineTo(new Vector2d(70.0/6 + 4, 33), Math.toRadians(90))
//                .lineToYConstantHeading(58.5)
//                .setReversed(true)
//                .splineTo(new Vector2d(-46, 24), Math.toRadians(-59 + 180))
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .splineTo(new Vector2d(0, 14), Math.toRadians(0))
//                .splineTo(new Vector2d(70.0/6 + 24 + 4, 33), Math.toRadians(90))
//                .lineToYConstantHeading(58.5)
//                .setReversed(true)
//                .splineTo(new Vector2d(-46, 24), Math.toRadians(-59 + 180))
//                .waitSeconds(1) // LAUNCH 3 BALLS
                .splineTo(new Vector2d(-44, 14), Math.toRadians(-90))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}