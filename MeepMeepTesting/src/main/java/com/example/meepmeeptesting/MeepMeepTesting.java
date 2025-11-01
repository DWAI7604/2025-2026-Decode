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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-57, 44, Math.toRadians(-55)))
//                .splineTo(new Vector2d(-50, 15), Math.toRadians(-90)) // moves out of way
                .waitSeconds(1)
                .splineTo(new Vector2d(-42, 30), Math.toRadians(-55))
                // LAUNCH HERE
                .waitSeconds(1)
                .splineTo(new Vector2d(-70.0/6, 28), Math.toRadians(90))
                .lineToY(49)
                .setReversed(true)
                .splineTo(new Vector2d(-42, 30), Math.toRadians(-55 + 180))
                // LAUNCH HERE
                .waitSeconds(1)
                .splineTo(new Vector2d(-35, 12), Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}