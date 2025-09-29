package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -60, 90))
                .strafeToLinearHeading(new Vector2d(-54,-53),Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(-48.5,-33), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-54,-53), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(-57.7,-33), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-54,-53), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(-67,-33), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-54,-53), Math.toRadians(225))
                .splineToConstantHeading(new Vector2d(-24,-6), Math.toRadians(-45))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}