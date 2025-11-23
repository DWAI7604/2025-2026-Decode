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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(63.5, 15 / 2.0, 0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(55, 18), -0.38), 0)
                .waitSeconds(1) // LAUNCH 3 BALLS
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/2, 28), Math.toRadians(90)), Math.toRadians(90))
                .lineToYConstantHeading(60)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(55, 18), -0.38), 0)
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(72.0/6, 28), Math.toRadians(90)), Math.toRadians(90))
//                .lineToYConstantHeading(60)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(new Vector2d(55, 18), -0.38), 0)
                .waitSeconds(1) // LAUNCH 3 BALLS
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(64.5, 38), Math.toRadians(90)), 0)
                .build());

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61, 44, Math.toRadians(-55)))
//                .splineTo(new Vector2d(-46, 24), Math.toRadians(-65))
//                .waitSeconds(1) // LAUNCH 3 BALLS
//                .splineTo(new Vector2d(-70.0/6, 24), Math.toRadians(90))
//                .lineToYConstantHeading(52)
//                .setReversed(true)
//                .splineTo(new Vector2d(-44, 24), Math.toRadians(-65 + 180))
//                .waitSeconds(1) // LAUNCH 3 BALLS
////                .splineToLinearHeading(new Pose2d(new Vector2d(-35, 12), Math.toRadians(-90)), 0) // LEAVE !!
//                .splineTo(new Vector2d(70.0/6, 36), Math.toRadians(90))
//                .lineToYConstantHeading(60)
//                .setReversed(true)
//                .splineTo(new Vector2d(-42, 24), Math.toRadians(-55 + 180))
//                .waitSeconds(1) // LAUNCH 3 BALLS
////                .splineTo(new Vector2d(0, 40), Math.toRadians(90)) // PARKING FOR 9 BALL ONLY
//                .splineTo(new Vector2d(-44, 14), Math.toRadians(-90))
//                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}