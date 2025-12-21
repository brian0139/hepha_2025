package com.example.meepmeeptesting.Aaron;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AarPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-51, -47, Math.toRadians(235)))// bottom left, facing up
                // Drive forward into the field
                .strafeTo(new Vector2d(-22, -25))
                .strafeToSplineHeading(new Vector2d(-11, -25), Math.toRadians(270))
                .strafeTo(new Vector2d(-11,-45))
                .waitSeconds(.5)
                .strafeToSplineHeading(new Vector2d(-52, -24), Math.toRadians(255))
                // Simulate shoot
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-43, -25), Math.toRadians(0))

                .strafeTo(new Vector2d(11, -25))
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(11,-45))
                .waitSeconds(.5)
                .strafeTo(new Vector2d(-52, -24))
                .turnTo(Math.toRadians(255))
                // Simulate shoot
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-43, -25), Math.toRadians(0))

                .strafeTo(new Vector2d(33, -25))
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(33, -45))
                .waitSeconds(.5)
                .strafeTo(new Vector2d(-52, -24))
                .turnTo(Math.toRadians(255))
                // Simulate shoot
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-43, -32), Math.toRadians(0))

                .strafeTo(new Vector2d(37, -33))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}