package com.example.meepmeeptesting.Stanley;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class testing {
    public static void main(String[] args) {
        final Vector2d shootingPos=new Vector2d(-34,23);
        final double shootingAngle=Math.toRadians(155);
        final double intakeFinishy =40;
        final double intakeStarty=13;
        final double waitTime=1;
        final double shootTime=1.5;
        final double row1XPos=-7;
        final double row2XPos=17;
        final double row3XPos=38;
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-57.5, 43.5, Math.toRadians(126)))
//                .stopAndAdd(new initHood())
//                .stopAndAdd(new SetHoodAngle(44))
//                            Start Flywheel 0
//                            .stopAndAdd(new SpinFlywheel(1640,50))
                .strafeToLinearHeading(shootingPos, shootingAngle)
                //Shooting Sequence 0
//                .stopAndAdd(new TurretAutoAimUntilAligned())
//                            .stopAndAdd(new transferUp())
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
                .waitSeconds(shootTime)
//                            //Stop Sequence 0
//                            .stopAndAdd(new StopFlywheel())
//                            .stopAndAdd(new transferOff())
//                            .stopAndAdd(new stopspindexer())
//                            .stopAndAdd(new StopIntake())
                //Start Intake Code 1
                .strafeToLinearHeading(new Vector2d(row1XPos, intakeStarty), Math.toRadians(360-270))
//                .stopAndAdd(new RunIntake())
                .waitSeconds(0.1)
//                .stopAndAdd(new startspindexer())
                .strafeTo(new Vector2d(row1XPos,intakeFinishy))

                //Stop Intake 1
                .waitSeconds(waitTime)
//                .stopAndAdd(new StopIntake())
//                .stopAndAdd(new stopspindexer())

                //Start Flywheel 1
//                            .stopAndAdd(new SpinFlywheel(1875,50))
                .strafeToLinearHeading(new Vector2d(row1XPos, intakeStarty-10), shootingAngle)
                //Shoot Sequence 1
//                .stopAndAdd(new TurretAutoAimUntilAligned())
//                            .stopAndAdd(new transferUp())
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
                .waitSeconds(shootTime)
                //Stop Sequence 1
//                            .stopAndAdd(new StopFlywheel())
//                            .stopAndAdd(new transferOff())
//                            .stopAndAdd(new stopspindexer())
//                            .stopAndAdd(new StopIntake())
//                            .stopAndAdd(new SetIntakePower(-1))
                .waitSeconds(0.3)
//                            .stopAndAdd(new StopIntake())
                //Start Intake 2
                .strafeToLinearHeading(new Vector2d(row2XPos, intakeStarty-5), Math.toRadians(360-270))
//                .stopAndAdd(new RunIntake())
//                .stopAndAdd(new startspindexer())
//                .strafeTo(new Vector2d(row2XPos, intakeFinishy))
                //Stop Intake 2
                .waitSeconds(waitTime)
//                .stopAndAdd(new StopIntake())
//                .stopAndAdd(new stopspindexer())
                .strafeToLinearHeading(new Vector2d(row2XPos, intakeFinishy-6), Math.toRadians(360-270))

                //Start Flywheel 2
//                            .stopAndAdd(new SetIntakePower(-1))
                            .waitSeconds(0.1)
//                            .stopAndAdd(new StopIntake())
//                            .stopAndAdd(new SpinFlywheel(1875,50))
                //.strafeToLinearHeading(new Vector2d(row1XPos, intakeStarty-10), shootingAngle)
                        .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(row1XPos, intakeStarty-10),shootingAngle),shootingAngle+Math.toRadians(90))

                //Shoot Sequence 2
//                .stopAndAdd(new TurretAutoAimUntilAligned())
//                            .stopAndAdd(new transferUp())
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
                            .waitSeconds(shootTime)
//                            .stopAndAdd(new SetIntakePower(-1))
//                .waitSeconds(0.3)
//                .stopAndAdd(new StopIntake())
//                //Stop Sequence 2
//                .stopAndAdd(new StopFlywheel())
//                .stopAndAdd(new transferOff())
//                .stopAndAdd(new stopspindexer())
//                .stopAndAdd(new StopIntake())
                //Start Intake 3
                .strafeToLinearHeading(new Vector2d(row3XPos, intakeStarty-10), Math.toRadians(360-270))
//                .stopAndAdd(new RunIntake())
//                .stopAndAdd(new startspindexer())
                .strafeTo(new Vector2d(row3XPos, intakeFinishy))
                //Stop Intake 3
                .waitSeconds(waitTime)
//                .stopAndAdd(new StopIntake())
//                .stopAndAdd(new stopspindexer())
                //Start Flywheel 3
//                            .stopAndAdd(new SetIntakePower(-1))
//                            .waitSeconds(0.1)
//                            .stopAndAdd(new StopIntake())
//                            .stopAndAdd(new SpinFlywheel(1875,50))
                .strafeToLinearHeading(new Vector2d(row1XPos, intakeStarty-10), shootingAngle)
                //Shoot Sequence 3
//                .stopAndAdd(new TurretAutoAimUntilAligned())
//                            .stopAndAdd(new transferUp())
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
//                            .stopAndAdd(new TurretAutoAimUntilAligned())
                .waitSeconds(shootTime)
                //Stop Sequence 3
//                .stopAndAdd(new StopFlywheel())
//                .stopAndAdd(new transferOff())
//                .stopAndAdd(new stopspindexer())
//                .stopAndAdd(new StopIntake())
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}