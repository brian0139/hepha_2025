package com.example.meepmeeptesting.Alvin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AlvPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 12, 0))
                // Drive forward from starting position to detect target Pixel
                .lineToX(-36)

                // Move to place Pixel on detection location
                .lineToXSplineHeading(-24, Math.toRadians(45))

                // Navigate to Pixel storage location
                .lineToXSplineHeading(-48, Math.toRadians(270))
                .forward(8)

                // Move to scoring area
                .lineToXSplineHeading(36, Math.toRadians(180))

                // Align with specific scoring section
                .strafeTo(new Vector2d(36, -36))

                // Park in designated end zone
                .lineToXSplineHeading(60, Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}