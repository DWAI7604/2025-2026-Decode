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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-39, 53, Math.toRadians(-90)))
                .waitSeconds(1)
                .splineTo(new Vector2d(-35, 30), Math.toRadians(-55)) // firing location
                .waitSeconds(1)
                // closest line
                .splineTo(new Vector2d(-70.0/6, 28), Math.toRadians(90))
                .lineToY(49)
                .setReversed(true)
                .splineTo(new Vector2d(-35, 30), Math.toRadians(180 - 55)) // firing location
                .waitSeconds(1)
                .setReversed(false)
                // second closest line
                .splineTo(new Vector2d(70.0/6, 30), Math.toRadians(90))
                .lineToY(49)
                .setReversed(true)
                .splineTo(new Vector2d(-35, 30), Math.toRadians(180 - 55)) // firing location
                .waitSeconds(1)
                .setReversed(false)
                // third closest line
                .splineTo(new Vector2d(70.0/2, 34), Math.toRadians(90))
                .lineToY(49)
                .setReversed(true)
                .splineTo(new Vector2d(-35, 30), Math.toRadians(180 - 55)) // firing location
                .waitSeconds(1)
                .setReversed(false)
                // parking
                .splineTo(new Vector2d(-35, 12), Math.toRadians(-90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}