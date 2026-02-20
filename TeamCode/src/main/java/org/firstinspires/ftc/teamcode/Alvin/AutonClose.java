package org.firstinspires.ftc.teamcode.Alvin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Alvin Auto Path", group = "Autonomous")
public class AutonClose extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-54, -23, Math.toRadians(90)));

        // Define shooting position and angle
        final Vector2d shootingPos = new Vector2d(-32, -32);
        final double shootingAngle = Math.toRadians(220);

        // Define classifier area where balls are collected
        final double intakeFinishy = -45;
        final double intakeStarty = -25.1;

        // Wait for start
        waitForStart();

        if (isStopRequested()) return;

        // Main action sequence for ball collection and shooting
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-54, -23, Math.toRadians(90)))
                        // First ball collection
                        .strafeToLinearHeading(new Vector2d(-12, intakeStarty), Math.toRadians(270))
                        .strafeTo(new Vector2d(-12, intakeFinishy))
                        .waitSeconds(1)  // Intake ball
                        .strafeToLinearHeading(shootingPos, shootingAngle)
                        .waitSeconds(2)  // Shoot for 2 seconds

                        // Second ball collection
                        .strafeToLinearHeading(new Vector2d(12, intakeStarty), Math.toRadians(270))
                        .strafeTo(new Vector2d(12, intakeFinishy))
                        .waitSeconds(1)  // Intake ball
                        .strafeToLinearHeading(shootingPos, shootingAngle)
                        .waitSeconds(2)  // Shoot for 2 seconds

                        // Third ball collection
                        .strafeToLinearHeading(new Vector2d(35, intakeStarty), Math.toRadians(270))
                        .strafeTo(new Vector2d(35, intakeFinishy))
                        .waitSeconds(1)  // Intake ball
                        .strafeToLinearHeading(shootingPos, shootingAngle)
                        .waitSeconds(2)  // Shoot for 2 seconds

                        // Move to final position
                        .splineToSplineHeading(new Pose2d(-60, -36, Math.toRadians(180)), Math.toRadians(180))
                        .build()
        );
    }
}