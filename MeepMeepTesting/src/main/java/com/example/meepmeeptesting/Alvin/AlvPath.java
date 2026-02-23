package com.example.meepmeeptesting.Alvin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AlvPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // ==================== CONSTANTS ====================
        // White triangle = start AND shoot position (top-right)
        final Pose2d beginPose     = new Pose2d(57.5, 43.5, Math.toRadians(180));
        final Vector2d shootingPos = new Vector2d(57.5, 43.5);  // same as start
        final double shootingAngle = Math.toRadians(180);

        // Ball cluster X columns (right to left)
        final double row3XPos      = 38;   // rightmost — nearest to white triangle
        final double row2XPos      = 16;   // center column
        final double row1XPos      = -9;   // left column

        // Intake sweep Y range (sweeping toward top wall)
        final double intakeStartY  = 46;
        final double intakeFinishY = 56;

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(beginPose)

                // ===== STEP 1: Shoot 3 preloaded balls from white triangle =====
                // (robot is already here, just shoot in place)

                // ===== STEP 2: Intake row 3 (nearest cluster, x=38) =====
                .strafeToLinearHeading(new Vector2d(row3XPos, intakeStartY), Math.toRadians(90))
                .strafeTo(new Vector2d(row3XPos, intakeFinishY))

                // ===== STEP 3: Return to white triangle and shoot =====
                .strafeToLinearHeading(shootingPos, shootingAngle)

                // ===== STEP 4: Intake row 2 (center, x=16) =====
                .strafeToLinearHeading(new Vector2d(row2XPos, intakeStartY), Math.toRadians(90))
                .strafeTo(new Vector2d(row2XPos, intakeFinishY))

                // ===== STEP 5: Return to white triangle and shoot =====
                .strafeToLinearHeading(shootingPos, shootingAngle)

                // ===== STEP 6: Intake row 1 (left, x=-9) =====
                .strafeToLinearHeading(new Vector2d(row1XPos, intakeStartY), Math.toRadians(90))
                .strafeTo(new Vector2d(row1XPos, intakeFinishY))

                // ===== STEP 7: Return to white triangle and shoot final balls =====
                .strafeToLinearHeading(shootingPos, shootingAngle)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
