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
                .setConstraints(50, 50, Math.PI, Math.PI, 18)
                .build();
        final Vector2d shootingPos=new Vector2d(-32,-32);
        final double shootingAngle=Math.toRadians(220);
        final boolean parkinginfar=false;
        final double intakeFinishy =-45;
        final double intakeStarty=-25;
        if (!parkinginfar) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61, -36, Math.toRadians(55)))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(-12, intakeStarty), Math.toRadians(270))
                    .strafeTo(new Vector2d(-12, intakeFinishy))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(12, intakeStarty), Math.toRadians(270))
                    .strafeTo(new Vector2d(12, intakeFinishy))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(35, intakeStarty), Math.toRadians(270))
                    .strafeTo(new Vector2d(35, intakeFinishy))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .splineToSplineHeading(new Pose2d(-62,-36,Math.toRadians(180)), Math.toRadians(180))
                    .build());
        }
        else{
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61, -36, Math.toRadians(180)))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(-12, intakeStarty), Math.toRadians(270))
                    .strafeTo(new Vector2d(-12, intakeFinishy))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(12, intakeStarty), Math.toRadians(270))
                    .strafeTo(new Vector2d(12, intakeFinishy))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .strafeToLinearHeading(new Vector2d(35, intakeStarty), Math.toRadians(270))
                    .strafeTo(new Vector2d(35, intakeFinishy))
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