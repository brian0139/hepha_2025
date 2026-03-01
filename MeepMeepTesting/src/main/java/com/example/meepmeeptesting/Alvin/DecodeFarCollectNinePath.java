package com.example.meepmeeptesting.Alvin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DecodeFarCollectNinePath {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        // ✅ Bottom of the RIGHTMOST white triangle (fixed)
        final double TRI_X = 55;
        final double TRI_Y = -16;
        final double TRI_H = Math.toRadians(180);
        final Pose2d beginPose = new Pose2d(TRI_X, TRI_Y, TRI_H);

        // ✅ Intake line Y’s (matches your screenshot)
        final double approachY = -34;
        final double intakeY = -52;

        // ✅ EXACT intake post X’s from your marked image
        final double leftPostX   = -7;
        final double middlePostX = 11;
        final double rightPostX  = 29;

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(beginPose)

                // ---------------- INTAKE 1 (LEFT POST) ----------------
                .strafeToLinearHeading(new Vector2d(leftPostX, approachY), Math.toRadians(-90))
                .strafeTo(new Vector2d(leftPostX, intakeY))
                .waitSeconds(0.2)

                // ✅ Return to rightmost white triangle after intake
                .strafeToLinearHeading(new Vector2d(TRI_X, TRI_Y), TRI_H)
                .waitSeconds(0.2)

                // ---------------- INTAKE 2 (MIDDLE POST) ----------------
                .strafeToLinearHeading(new Vector2d(middlePostX, approachY), Math.toRadians(-90))
                .strafeTo(new Vector2d(middlePostX, intakeY))
                .waitSeconds(0.2)

                // ✅ Return to rightmost white triangle after intake
                .strafeToLinearHeading(new Vector2d(TRI_X, TRI_Y), TRI_H)
                .waitSeconds(0.2)

                // ---------------- INTAKE 3 (RIGHT POST) ----------------
                .strafeToLinearHeading(new Vector2d(rightPostX, approachY), Math.toRadians(-90))
                .strafeTo(new Vector2d(rightPostX, intakeY))
                .waitSeconds(0.2)

                // ✅ Return to rightmost white triangle after intake
                .strafeToLinearHeading(new Vector2d(TRI_X, TRI_Y), TRI_H)
                .waitSeconds(0.2)

                // Park (keep/change as you want)
                .strafeToLinearHeading(new Vector2d(50, -30), Math.toRadians(-90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}