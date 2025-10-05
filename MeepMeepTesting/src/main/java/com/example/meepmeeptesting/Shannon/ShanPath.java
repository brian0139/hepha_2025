package com.example.meepmeeptesting.Shannon;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ShanPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 0, 0))
                // add your own code now OR ELSE!!!!!
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static class driveTrain {
        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(750);

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .build();

            // start at red square
            Pose2d startPose = new Pose2d(36, -36, Math.toRadians(180));
            myBot.setPose(startPose);

            myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                    // collect cycle 1
                    .splineToConstantHeading(new Vector2d(31.92, -34.50), Math.toRadians(180))
                    .waitSeconds(0.4) // intake

                    // collect cycle 2
                    .splineToConstantHeading(new Vector2d(18.09, -21.40), Math.toRadians(180))
                    .waitSeconds(0.4) // intake

                    // collect cycle 3
                    .splineToConstantHeading(new Vector2d(6.01, -23.92), Math.toRadians(180))
                    .waitSeconds(0.4) // 3 balls atm

                    // scoring
                    .splineToConstantHeading(new Vector2d(-36, 36), Math.toRadians(135)) // red triangle approx ~
                    .waitSeconds(1.0) // scoring

                    // return to og pos, add to code. change cycles to 3 balls, not 1
                    .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(-45))
                    .splineToConstantHeading(new Vector2d(36, -36), Math.toRadians(180))
                    .waitSeconds(0.2)

                    .build()
            );

            meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
}