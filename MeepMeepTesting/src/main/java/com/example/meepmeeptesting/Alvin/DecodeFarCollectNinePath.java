package com.example.meepmeeptesting.Alvin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DecodeFarCollectNinePath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        final Pose2d beginPose = new Pose2d(57.5, -43.5, Math.toRadians(-54));
        final Vector2d shootingPos = new Vector2d(34, -23);
        final double shootingAngle = Math.toRadians(-40);

        final double intakeStartY = -13;
        final double intakeFinishY = -50;
        final double row1XPos = 9;
        final double row2XPos = -16;
        final double row3XPos = -38;

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(beginPose)
                // Preload shooting setup
                .strafeToLinearHeading(shootingPos, shootingAngle)
                .waitSeconds(1.5)

                // Cycle 1: lane 1 intake and return to shoot
                .strafeToLinearHeading(new Vector2d(row1XPos, intakeStartY), Math.toRadians(-80))
                .strafeTo(new Vector2d(row1XPos, intakeFinishY - 4))
                .strafeToLinearHeading(new Vector2d(row1XPos, intakeStartY + 10), shootingAngle)
                .waitSeconds(1.5)

                // Cycle 2: lane 2 intake and spline return
                .strafeToLinearHeading(new Vector2d(row2XPos, intakeStartY + 5), Math.toRadians(90))
                .strafeTo(new Vector2d(row2XPos, intakeFinishY - 5))
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(new Vector2d(row1XPos, intakeStartY + 10), shootingAngle),
                        shootingAngle - Math.toRadians(90))
                .setReversed(false)
                .waitSeconds(1.5)

                // Cycle 3: lane 3 intake and final return to shoot
                .strafeToLinearHeading(new Vector2d(row3XPos, intakeStartY + 10), Math.toRadians(90))
                .strafeTo(new Vector2d(row3XPos, intakeFinishY - 5))
                .strafeToLinearHeading(new Vector2d(row1XPos, intakeStartY + 10), shootingAngle)
                .waitSeconds(1.5)

                // Park
                .strafeToLinearHeading(new Vector2d(48, -60), Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
