package org.firstinspires.ftc.teamcode.Stanley.autonPath;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Brian.spindexer;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class AutonRedPath extends LinearOpMode {
    Servo spindexerServo=null;
    DcMotor intakeMotor=null;
    //classes
    spindexer spindexerOperator=null;
    @Override
    public void runOpMode() throws InterruptedException{
        spindexerServo=hardwareMap.servo.get("spindexerServo");
        intakeMotor=hardwareMap.dcMotor.get("intake");
        spindexerOperator=new spindexer(spindexerServo);
        Pose2d beginPose=new Pose2d(-57.5, -43.5, Math.toRadians(54));
        MecanumDrive drive=new MecanumDrive(hardwareMap,beginPose);
        final Vector2d shootingPos=new Vector2d(-34,-23);
        final double shootingAngle=Math.toRadians(225);
        final double intakeFinishy =-36;
        final double intakeStarty=-13;
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(new Vector2d(-10, intakeStarty), Math.toRadians(270))
                        .stopAndAdd(new spinSpindexer(spindexerOperator,0))
                        .stopAndAdd(new intakeStart(intakeMotor,1))
                    .strafeTo(new Vector2d(-10, intakeFinishy))
                        .stopAndAdd(new intakeStop(intakeMotor))
                        .stopAndAdd(new spinSpindexer(spindexerOperator,1))
                        .waitSeconds(1)
                        .stopAndAdd(new intakeStart(intakeMotor,0.8))
                    .strafeTo(new Vector2d(-10,intakeFinishy-10))
                        .waitSeconds(1)
                        .stopAndAdd(new intakeStop(intakeMotor))
                        .stopAndAdd(new spinSpindexer(spindexerOperator,2))
                        .waitSeconds(1)
                        .stopAndAdd(new intakeStart(intakeMotor,1))
                        .waitSeconds(1)
                        .stopAndAdd(new intakeStop(intakeMotor))

//                    .waitSeconds(3)
//                    .strafeToLinearHeading(shootingPos, shootingAngle)
//                    .waitSeconds(3)
//                    .strafeToLinearHeading(new Vector2d(10, intakeStarty+7), Math.toRadians(275))
//                    .strafeTo(new Vector2d(10, intakeFinishy+3))
//                    .waitSeconds(3)
//                    .strafeToLinearHeading(shootingPos, shootingAngle)
//                    .waitSeconds(3)
//                    .strafeToLinearHeading(new Vector2d(35, intakeStarty+10), Math.toRadians(270))
//                    .strafeTo(new Vector2d(35, intakeFinishy+10))
//                    .waitSeconds(3)
//                    .strafeToLinearHeading(shootingPos, shootingAngle)
//                    .waitSeconds(3)
                    .build());
    }
    public class intakeStart implements Action{
        DcMotor intake;
        double power;
        public intakeStart(DcMotor intake, double power){
            this.intake=intake;
            this.power=power;
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            this.intake.setPower(this.power);
            return false;
        }
    }
    public class spinSpindexer implements Action{
        spindexer spindexerOperator;
        int position;
        public spinSpindexer(spindexer spindexerOperator,int position){
            this.spindexerOperator=spindexerOperator;
            this.position=position;
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            this.spindexerOperator.rotateSpindexerInput(this.position);
            return false;
        }
    }
    public class intakeStop implements Action{
        DcMotor intake;
        public intakeStop(DcMotor intake){
            this.intake=intake;
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            this.intake.setPower(0);
            return false;
        }
    }
}
