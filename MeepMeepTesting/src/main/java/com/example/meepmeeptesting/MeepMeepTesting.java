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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, -60, Math.toRadians(90)))

                // Leg 1: Drive forward toward the wing
                .splineTo(new Vector2d(-36, -12), Math.toRadians(90))

                // Leg 2: Sweep right across the center of the field
                .splineTo(new Vector2d(0, -24), Math.toRadians(0))

                // Leg 3: Curve up toward the backdrop / scoring area
                .splineTo(new Vector2d(36, -12), Math.toRadians(45))

                // Leg 4: Loop down the right side of the field
                .splineTo(new Vector2d(48, -36), Math.toRadians(-45))

                // Leg 5: Sweep back left along the bottom of the field
                .splineTo(new Vector2d(24, -56), Math.toRadians(180))

                // Leg 6: Arc back toward the starting side, closing the loop
                .splineTo(new Vector2d(-12, -48), Math.toRadians(135))

                // Leg 7: Final spline back toward parking zone
                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))

                // Park
                .splineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(180)), Math.toRadians(180))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}