package org.firstinspires.ftc.teamcode.Stanley.autonPath;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class AutonRedPath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Pose2d beginPose=new Pose2d(-57.5, -43.5, Math.toRadians(54));
        MecanumDrive drive=new MecanumDrive(hardwareMap,beginPose);
        final Vector2d shootingPos=new Vector2d(-23,-23);
        final double shootingAngle=Math.toRadians(225);
        final double intakeFinishy =-36;
        final double intakeStarty=-18;
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .waitSeconds(3)
                    .strafeToLinearHeading(new Vector2d(-12, intakeStarty), Math.toRadians(270))
                    .strafeTo(new Vector2d(-12, intakeFinishy))
                    .waitSeconds(3)
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .waitSeconds(3)
                    .strafeToLinearHeading(new Vector2d(10, intakeStarty+7), Math.toRadians(275))
                    .strafeTo(new Vector2d(10, intakeFinishy+3))
                    .waitSeconds(3)
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .waitSeconds(3)
                    .strafeToLinearHeading(new Vector2d(35, intakeStarty+10), Math.toRadians(270))
                    .strafeTo(new Vector2d(35, intakeFinishy+10))
                    .waitSeconds(3)
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .waitSeconds(3)
                    .build());
    }
}
