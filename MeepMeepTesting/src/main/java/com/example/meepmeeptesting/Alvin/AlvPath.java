package com.example.meepmeeptesting.Alvin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AlvPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        // Set bot constraints
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(80, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Define shooting/basket position (red structure on right side in video)
        final Vector2d basketPos = new Vector2d(54, -54);  // High basket position
        final double basketAngle = Math.toRadians(45);      // Face basket

        // Define intake positions - 4 rows of 3 balls each (12 total)
        final double row1Y = -48;  // First row
        final double row2Y = -38;  // Second row
        final double row3Y = -28;  // Third row
        final double row4Y = -18;  // Fourth row (closest to center)

        // X positions for the 3 balls in each row
        final double col1X = -48;  // Left column
        final double col2X = -36;  // Middle column
        final double col3X = -24;  // Right column

        // Main action sequence - 12 balls in 30 seconds
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                // === CYCLE 1: Collect 3 balls from row 1 ===
                .strafeToLinearHeading(new Vector2d(col1X, row1Y), Math.toRadians(0))
                .waitSeconds(0.3)  // Intake ball 1

                .strafeTo(new Vector2d(col2X, row1Y))
                .waitSeconds(0.3)  // Intake ball 2

                .strafeTo(new Vector2d(col3X, row1Y))
                .waitSeconds(0.3)  // Intake ball 3

                // Score at basket
                .strafeToLinearHeading(basketPos, basketAngle)
                .waitSeconds(1.5)  // Shoot 3 balls

                // === CYCLE 2: Collect 3 balls from row 2 ===
                .strafeToLinearHeading(new Vector2d(col1X, row2Y), Math.toRadians(0))
                .waitSeconds(0.3)  // Intake ball 4

                .strafeTo(new Vector2d(col2X, row2Y))
                .waitSeconds(0.3)  // Intake ball 5

                .strafeTo(new Vector2d(col3X, row2Y))
                .waitSeconds(0.3)  // Intake ball 6

                // Score at basket
                .strafeToLinearHeading(basketPos, basketAngle)
                .waitSeconds(1.5)  // Shoot 3 balls

                // === CYCLE 3: Collect 3 balls from row 3 ===
                .strafeToLinearHeading(new Vector2d(col1X, row3Y), Math.toRadians(0))
                .waitSeconds(0.3)  // Intake ball 7

                .strafeTo(new Vector2d(col2X, row3Y))
                .waitSeconds(0.3)  // Intake ball 8

                .strafeTo(new Vector2d(col3X, row3Y))
                .waitSeconds(0.3)  // Intake ball 9

                // Score at basket
                .strafeToLinearHeading(basketPos, basketAngle)
                .waitSeconds(1.5)  // Shoot 3 balls

                // === CYCLE 4: Collect 3 balls from row 4 ===
                .strafeToLinearHeading(new Vector2d(col1X, row4Y), Math.toRadians(0))
                .waitSeconds(0.3)  // Intake ball 10

                .strafeTo(new Vector2d(col2X, row4Y))
                .waitSeconds(0.3)  // Intake ball 11

                .strafeTo(new Vector2d(col3X, row4Y))
                .waitSeconds(0.3)  // Intake ball 12

                // Score final 3 balls at basket
                .strafeToLinearHeading(basketPos, basketAngle)
                .waitSeconds(1.5)  // Shoot 3 balls

                // Park in observation zone
                .strafeToLinearHeading(new Vector2d(-36, -60), Math.toRadians(90))
                .build());

        // Set up the MeepMeep simulation
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}