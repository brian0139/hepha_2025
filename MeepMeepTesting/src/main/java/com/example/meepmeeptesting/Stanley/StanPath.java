package com.example.meepmeeptesting.Stanley;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class StanPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        Vector2d shootingPos=new Vector2d(-32,-32);
        double shootingAngle=Math.toRadians(220);
        final boolean parkinginfar=true;
        if (parkinginfar) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61, -36, Math.toRadians(180)))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(-12, -30), Math.toRadians(90))
                    .strafeTo(new Vector2d(-12, -45))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(12, -30), Math.toRadians(90))
                    .strafeTo(new Vector2d(12, -45))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(35, -30), Math.toRadians(90))
                    .strafeTo(new Vector2d(35, -45))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .splineToSplineHeading(new Pose2d(-62,-36,Math.toRadians(180)), Math.toRadians(180))
                    .build());
        }
        else{
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61, -36, Math.toRadians(180)))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(-12, -30), Math.toRadians(90))
                    .strafeTo(new Vector2d(-12, -45))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(12, -30), Math.toRadians(90))
                    .strafeTo(new Vector2d(12, -45))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(35, -30), Math.toRadians(90))
                    .strafeTo(new Vector2d(35, -45))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .setReversed(true)
                    .splineTo(new Vector2d(61, -9), Math.toRadians(0))
                    .build());
        }
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}