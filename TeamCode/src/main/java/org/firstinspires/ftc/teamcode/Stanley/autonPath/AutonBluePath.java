package org.firstinspires.ftc.teamcode.Stanley.autonPath;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Brian.spindexer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.outtake;

@Autonomous
public class AutonBluePath extends LinearOpMode {
    Servo spindexerServo=null;
    DcMotor intakeMotor=null;
    Servo transfer=null;
    DcMotorEx flywheel=null;
    CRServo hood=null;
    //classes
    spindexer spindexerOperator=null;
    @Override
    public void runOpMode() throws InterruptedException{
        spindexerServo=hardwareMap.servo.get("spindexerServo");
        intakeMotor=hardwareMap.dcMotor.get("intake");
        flywheel=(DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer=hardwareMap.servo.get("transferServo");
        hood=hardwareMap.crservo.get("hoodServo");
        spindexerOperator=new spindexer(spindexerServo);
        Pose2d beginPose=new Pose2d(-57.5, -43.5, Math.toRadians(54));
        MecanumDrive drive=new MecanumDrive(hardwareMap,beginPose);
        final Vector2d shootingPos=new Vector2d(-34,-23);
        final double shootingAngle=Math.toRadians(225);
        final double intakeFinishy =-36;
        final double intakeStarty=-13;
        //TODO:get values for shooting hood angle and flywheel speed
        final double hoodAngle;
        final double flywheelSpeed;

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(new Vector2d(-15, intakeStarty), Math.toRadians(270))
//                        .stopAndAdd(new spinSpindexer(spindexerOperator,0))
                        .stopAndAdd(new intakeStart(intakeMotor,1))
                    .strafeTo(new Vector2d(-15, intakeFinishy))
                        .stopAndAdd(new intakeStop(intakeMotor))
//                        .stopAndAdd(new spinSpindexer(spindexerOperator,1))
                        .waitSeconds(1)
                        .stopAndAdd(new intakeStart(intakeMotor,0.8))
                    .strafeTo(new Vector2d(-15,intakeFinishy-10))
                        .waitSeconds(1)
//                        .stopAndAdd(new intakeStop(intakeMotor))
                        .stopAndAdd(new spinSpindexer(spindexerOperator,2,false))
                        .waitSeconds(0.5)
                        .stopAndAdd(new intakeStart(intakeMotor,1))
//                        .waitSeconds(0.5)
//                        .stopAndAdd(new intakeStop(intakeMotor))

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
    //trajectory generation function
    public Action intake(double endY, double x, double ramY, Pose2d startPose, MecanumDrive drive){
        Action intakeTrajectory=drive.actionBuilder(startPose)
                //spin to 0 intake
                .stopAndAdd(new spinSpindexer(spindexerOperator,0,false))
                //start intake
                .stopAndAdd(new intakeStart(intakeMotor,1))
                //strafe forwards
                .strafeTo(new Vector2d(x,endY))
                //stop intake
                .stopAndAdd(new intakeStop(intakeMotor))
                //spin to 1 intake
                .stopAndAdd(new spinSpindexer(spindexerOperator,1,false))
                //strafe
                .build();
        return intakeTrajectory;
    }
    //action classes
    public class intakeStart implements Action{
        DcMotor intake;
        double power;
        long starttime;
        boolean started=false;
        public intakeStart(DcMotor intake, double power){
            this.intake=intake;
            this.power=power;
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            this.intake.setPower(this.power);
            this.starttime=System.currentTimeMillis();
            this.started=true;
            return false;
        }
    }
    public class spinSpindexer implements Action{
        spindexer spindexerOperator;
        int position;
        //false=intake,true=outtake
        boolean inout=false;
        public spinSpindexer(spindexer spindexerOperator,int position,boolean inout){
            this.spindexerOperator=spindexerOperator;
            this.position=position;
            this.inout=inout;
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            if (!inout) {
                this.spindexerOperator.rotateSpindexerInput(this.position);
            }
            else{
                this.spindexerOperator.rotateSpindexerOutput(this.position);
            }
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
    public class moveHood implements Action{
        double angle;
        CRServo hood;
        outtake outtakeOperator;
        boolean wait=false;
        public moveHood(double angle, CRServo hood,boolean wait){
            this.angle=angle;
            this.hood=hood;
            this.wait=wait;
            this.outtakeOperator=new outtake(hardwareMap,null,null,null,null,null,null,hood,null,false);
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            if (this.wait) {
                return this.outtakeOperator.setHood(this.angle);
            }
            else{
                this.outtakeOperator.setHood(this.angle);
                return false;
            }
        }
    }
    public class spinFlywheel implements Action{
        DcMotorEx flywheel;
        int targetSpeed;
        int tolerance=5;
        outtake outtakeOperator;
        public spinFlywheel(DcMotorEx flywheel,int targetSpeed) {
            this.flywheel = flywheel;
            this.targetSpeed=targetSpeed;
            this.outtakeOperator = new outtake(hardwareMap, flywheel, null, null, null, null, null, null, null,false);
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            this.outtakeOperator.spin_flywheel(this.targetSpeed,this.tolerance);
            return false;
        }
    }
    public class moveTransfer implements Action{
        Servo transfer;
        //true=up, false=down
        boolean position;
        outtake outtakeOperator;
        public moveTransfer(Servo transfer, boolean position){
            this.transfer=transfer;
            this.position=position;
            this.outtakeOperator=new outtake(hardwareMap,null,null,null,null,null,null,null,transfer,false);
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            if (this.position){
                telemetry.addLine("transferUp");
                telemetry.update();
                this.outtakeOperator.transferUp();
            }
            else{
                telemetry.addLine("transferDown");
                telemetry.update();
                this.outtakeOperator.transferDown();
            }
            return false;
        }
    }

}
