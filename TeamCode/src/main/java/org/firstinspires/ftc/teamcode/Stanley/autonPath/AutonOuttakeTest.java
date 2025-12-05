package org.firstinspires.ftc.teamcode.Stanley.autonPath;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Brian.spindexer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.outtake;

@Autonomous
public class AutonOuttakeTest extends LinearOpMode {
    Servo spindexerServo=null;
    Servo transfer=null;
    DcMotor intakeMotor=null;
    DcMotorEx flywheel=null;
    //classes
    spindexer spindexerOperator=null;
    @Override
    public void runOpMode() throws InterruptedException{
        spindexerServo=hardwareMap.servo.get("spindexerServo");
        intakeMotor=hardwareMap.dcMotor.get("intake");
        flywheel=(DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        transfer=hardwareMap.servo.get("transferServo");
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
                        .stopAndAdd(new spinSpindexer(spindexerOperator,0,false))
                        .stopAndAdd(new spinSpindexer(spindexerOperator,1,false))
                        .stopAndAdd(new spinSpindexer(spindexerOperator,2,false))
                        .stopAndAdd(new intakeStart(intakeMotor,1,10000))
                        .stopAndAdd(new spinSpindexer(spindexerOperator,0,true))
                        .waitSeconds(1)
                        .stopAndAdd(new spinFlywheel(flywheel,500,true))
                        .stopAndAdd(new moveTransfer(transfer,true))
                        .waitSeconds(0.5)
                        .stopAndAdd(new moveTransfer(transfer,false))
                        .stopAndAdd(new spinSpindexer(spindexerOperator,1,true))
                        .stopAndAdd(new moveTransfer(transfer,true))
                        .waitSeconds(0.5)
                        .stopAndAdd(new moveTransfer(transfer,false))
                        .stopAndAdd(new spinSpindexer(spindexerOperator,2,true))
                        .stopAndAdd(new moveTransfer(transfer,true))
                        .waitSeconds(0.5)
                        .stopAndAdd(new moveTransfer(transfer,false))
                        .stopAndAdd(new spinFlywheel(flywheel,0,false))
                        .stopAndAdd(new moveTransfer(transfer,false))
                    .build());
    }

    //action classes
    public class intakeStart implements Action{
        DcMotor intake;
        double power;
        double timems;
        long starttime;
        boolean started=false;
        public intakeStart(DcMotor intake, double power, double timems){
            this.intake=intake;
            this.power=power;
            this.timems=timems;
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            if (!started) {
                this.intake.setPower(this.power);
                this.starttime=System.currentTimeMillis();
                this.started=true;
                return false;
            }
            if (this.timems < 0){
                return true;
            }
            if (System.currentTimeMillis()-this.starttime>=this.timems){
                this.intake.setPower(0);
                this.started=false;
                return true;
            }
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
        boolean wait=false;
        int targetSpeed;
        int tolerance=5;
        outtake outtakeOperator;
        public spinFlywheel(DcMotorEx flywheel,int targetSpeed, boolean wait) {
            this.flywheel = flywheel;
            this.wait = wait;
            this.targetSpeed=targetSpeed;
            this.outtakeOperator = new outtake(hardwareMap, flywheel, null, null, null, null, null, null, null,false);
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            if (this.wait){
                return this.outtakeOperator.spin_flywheel(this.targetSpeed,this.tolerance);
            }
            else{
                this.outtakeOperator.spin_flywheel(this.targetSpeed,this.tolerance);
                return false;
            }
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
                this.outtakeOperator.transferUp();
            }
            else{
                this.outtakeOperator.transferDown();
            }
            return false;
        }
    }

}
