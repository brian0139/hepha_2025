package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.PID;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.holdPosition;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;

@TeleOp
public class tuning extends LinearOpMode {
    CRServo hood=null;
    AnalogInput hoodSensor=null;
    outtakeV2 outtakeOperator=null;
    double change=0.1;
    int x=0;
    //TODO:Get real value+sync with outtakeV2 value
    double angle=60;
    boolean correctingtoggle=false;
    //FTC dashboard telemetry
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;

    @Override
    public void runOpMode(){
        hood=hardwareMap.get(CRServo.class,"hoodServo");
        hoodSensor=hardwareMap.get(AnalogInput.class,"hoodAnalog");
        outtakeOperator=new outtakeV2(hardwareMap,null,null,"Red",null,null,null,null,hood,hoodSensor,null,false);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.yWasPressed()) correctingtoggle=!correctingtoggle;
            //shift speed
            if (gamepad1.rightBumperWasPressed()){
                change*=10;
            }else if (gamepad1.leftBumperWasPressed()){
                change/=10;
            }
            telemetry.addData("Change",change);
            //selection
            if (gamepad1.dpadLeftWasPressed()){
                x--;
                if (x<0){
                    x=2;
                }
            }
            if (gamepad1.dpadRightWasPressed()){
                x++;
                if (x>2){
                    x=0;
                }
            }
            if (gamepad1.dpadUpWasPressed()){
                outtakeOperator.Kh[x]+=change;
            }
            if (gamepad1.dpadDownWasPressed()){
                outtakeOperator.Kh[x]-=change;
            }
            if (gamepad1.leftStickButtonWasPressed()){
                angle+=change;
            }
            if (gamepad1.rightStickButtonWasPressed()){
                angle-=change;
            }
            String line1="Kx: ";
            for (int i=0;i<=2;i++){
                if (i==x){
                    line1+="{";
                }
                outtakeOperator.Kh[i]=(double) Math.round(outtakeOperator.Kh[i] * Math.pow(10, 5)) / Math.pow(10, 5);
                line1+=outtakeOperator.Kh[i];
                if (i==x){
                    line1+="}";
                }
                line1+=", ";
            }
            if (gamepad1.xWasPressed()){
                outtakeOperator.hoodPID=new PID(outtakeOperator.Kh[0],outtakeOperator.Kh[1],outtakeOperator.Kh[2]);
            }
            boolean atTarget=false;
            if (correctingtoggle){
                atTarget=outtakeOperator.setHood(angle);
            }else{
                outtakeOperator.stopHood();
            }
            outtakeOperator.updateHoodAngle();

            telemetry.addLine(line1);
            telemetry.addData("Holding",correctingtoggle);
            telemetry.addData("Target",angle);
            telemetry.addData("Current",outtakeOperator.hoodAngle);
            telemetry.addData("Power",outtakeOperator.hoodPID.power);
            telemetry.addData("Offset(rotations)",angle/outtakeOperator.servoDegPerRot-outtakeOperator.hoodAngle);
            telemetry.addData("AtTarget",atTarget);
            telemetry.update();
            dashboardTelemetry.addLine(line1);
            dashboardTelemetry.addData("Holding",correctingtoggle);
            dashboardTelemetry.addData("Target",angle);
            dashboardTelemetry.addData("Current",outtakeOperator.hoodAngle);
            dashboardTelemetry.addData("Power",outtakeOperator.hoodPID.power);
            dashboardTelemetry.addData("Offset(rotations)",angle/outtakeOperator.servoDegPerRot-outtakeOperator.hoodAngle);
            dashboardTelemetry.addData("AtTarget",atTarget);
            dashboardTelemetry.update();
        }
    }
}
