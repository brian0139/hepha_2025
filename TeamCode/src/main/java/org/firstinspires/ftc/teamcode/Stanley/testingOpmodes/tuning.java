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
        telemetry = dashboard.getTelemetry();
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
            String line1="Kx: ";
            for (int i=0;i<=2;i++){
                if (i==x){
                    line1+="{";
                }
                holdPositionOperator.Kx[i]=(double) Math.round(holdPositionOperator.Kx[i] * Math.pow(10, 5)) / Math.pow(10, 5);
                line1+=holdPositionOperator.Kx[i];
                if (i==x && y==0){
                    line1+="}";
                }
                line1+=", ";
            }
//            telemetry.addData("X",x);
//            telemetry.addData("Y",y);
            if (gamepad1.xWasPressed()){
                outtakeOperator.hoodPID=new PID(outtakeOperator.Kh[0],outtakeOperator.Kh[1],outtakeOperator.Kh[2]);
            }
            if (correctingtoggle){
                holdPositionOperator.hold();
            }else{
                holdPositionOperator.stop();
            }
            holdPositionOperator.updateCurrentPosition();

            telemetry.addLine(line1);
            telemetry.addLine(line2);
            telemetry.addLine(line3);
            telemetry.addData("powerX",-holdPositionOperator.powerX);
            telemetry.addData("powerY",holdPositionOperator.powerY);
            telemetry.addData("powerT",holdPositionOperator.powerT);
            telemetry.addData("Holding",correctingtoggle);
            telemetry.addData("XOffset",holdPositionOperator.initialPosition.position.x-holdPositionOperator.currentPosition.position.x);
            telemetry.addData("YOffset",holdPositionOperator.initialPosition.position.y-holdPositionOperator.currentPosition.position.y);
            telemetry.addData("TOffset",holdPositionOperator.initialPosition.heading.imag * holdPositionOperator.currentPosition.heading.real - holdPositionOperator.initialPosition.heading.real * holdPositionOperator.currentPosition.heading.imag);
            telemetry.update();
        }
    }
}
