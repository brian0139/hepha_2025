package com.example.meepmeeptesting.Alvin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AlvPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        // Set bot constraints (adjust max velocity and acceleration as necessary)
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(80, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Define shooting position and angle
        final Vector2d shootingPos = new Vector2d(-32, -32);  // Set the shooting position
        final double shootingAngle = Math.toRadians(220);      // Set shooting angle

        // Define classifier area where balls are collected
        final Vector2d classifierArea = new Vector2d(-12, -25);  // Adjust position of classifier
        final double intakeFinishy = -45;  // Position where balls are collected
        final double intakeStarty = -25.1;  // Position for approaching the balls

        // Define the number of balls to collect (15 balls)
        int numBalls = 15;

        // Main action sequence for ball collection and shooting
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-54, -23, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-12, intakeStarty), Math.toRadians(270))  // Move to first ball in classifier
                .strafeTo(new Vector2d(-12, intakeFinishy))  // Collect first ball
                .waitSeconds(1)  // Wait for collection
                .strafeToLinearHeading(shootingPos, shootingAngle)  // Move to shooting position
                .waitSeconds(1)  // Shoot first ball
                .strafeToLinearHeading(new Vector2d(12, intakeStarty), Math.toRadians(270))  // Move to second ball
                .strafeTo(new Vector2d(12, intakeFinishy))  // Collect second ball
                .waitSeconds(1)  // Wait for collection
                .strafeToLinearHeading(shootingPos, shootingAngle)  // Move to shooting position
                .waitSeconds(1)  // Shoot second ball
                .strafeToLinearHeading(new Vector2d(35, intakeStarty), Math.toRadians(270))  // Move to third ball
                .strafeTo(new Vector2d(35, intakeFinishy))  // Collect third ball
                .waitSeconds(1)  // Wait for collection
                .strafeToLinearHeading(shootingPos, shootingAngle)  // Move to shooting position
                .waitSeconds(1)  // Shoot third ball
                .splineToSplineHeading(new Pose2d(-60, -36, Math.toRadians(180)), Math.toRadians(180))  // Move to next ball area
                .build());

        // Set up the MeepMeep simulation with background and entities
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
