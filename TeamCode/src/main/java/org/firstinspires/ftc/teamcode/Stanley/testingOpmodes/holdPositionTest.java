package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.holdPosition;

@TeleOp
public class holdPositionTest extends LinearOpMode {
    holdPosition holdPositionOperator=null;
    double change=1;
    int x=0;
    int y=0;
    boolean correctingtoggle=false;
    MecanumDrive drive;

    @Override
    public void runOpMode(){
        drive=new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(0,0),0));
        holdPositionOperator=new holdPosition(drive);
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
                    y--;
                }
                if (y<0){
                    y=2;
                }
            }
            if (gamepad1.dpadRightWasPressed()){
                x++;
                if (x>2){
                    x=0;
                    y++;
                }
                if (y>2){
                    y=0;
                }
            }
            if (gamepad1.dpadUpWasPressed()){
                if (y==0){
                    holdPositionOperator.Kx[x]+=change;
                } else if (y == 1) {
                    holdPositionOperator.Ky[x]+=change;
                }else{
                    holdPositionOperator.Kt[x]+=change;
                }
            }
            if (gamepad1.dpadDownWasPressed()){
                if (y==0){
                    holdPositionOperator.Kx[x]-=change;
                } else if (y == 1) {
                    holdPositionOperator.Ky[x]-=change;
                }else{
                    holdPositionOperator.Kt[x]-=change;
                }
            }
            String line1="Kx: ";
            String line2="Ky:";
            String line3="Kt:";
            for (int i=0;i<=2;i++){
                if (i==x && y==0){
                    line1+="{";
                }
                holdPositionOperator.Kx[i]=(double) Math.round(holdPositionOperator.Kx[i] * Math.pow(10, 5)) / Math.pow(10, 5);
                line1+=holdPositionOperator.Kx[i];
                if (i==x && y==0){
                    line1+="}";
                }
                line1+=", ";
            }
            for (int i=0;i<=2;i++){
                if (i==x && y==1){
                    line2+="{";
                }
                holdPositionOperator.Ky[i]=(double) Math.round(holdPositionOperator.Kx[i] * Math.pow(10, 5)) / Math.pow(10, 5);
                line2+=holdPositionOperator.Ky[i];
                if (i==x && y==1){
                    line2+="}";
                }
                line2+=", ";
            }
            for (int i=0;i<=2;i++){
                if (i==x && y==2){
                    line3+="{";
                }
                holdPositionOperator.Kt[i]=(double) Math.round(holdPositionOperator.Kx[i] * Math.pow(10, 5)) / Math.pow(10, 5);
                line3+=holdPositionOperator.Kt[i];
                if (i==x && y==2){
                    line3+="}";
                }
                line3+=", ";
            }
            telemetry.addData("X",x);
            telemetry.addData("Y",y);

            telemetry.addLine(line1);
            telemetry.addLine(line2);
            telemetry.addLine(line3);
            telemetry.addData("XOffset",holdPositionOperator.initialPosition.position.x-holdPositionOperator.currentPosition.position.x);
            telemetry.addData("YOffset",holdPositionOperator.initialPosition.position.y-holdPositionOperator.currentPosition.position.y);
            telemetry.addData("TOffset",holdPositionOperator.initialPosition.heading.imag * holdPositionOperator.currentPosition.heading.real - holdPositionOperator.initialPosition.heading.real * holdPositionOperator.currentPosition.heading.imag);
            telemetry.update();
        }
    }
}
