package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;

public class spindexerIntakeSpeedPairTest extends LinearOpMode {
    DcMotor intake;
    CRServo spindexer;
    //FTC dashboard telemetry
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;

    boolean correctingtoggle=false;
    double change=0.1;
    int x=0;
    //0=intake,1=spindexer
    double[] values={0.1,0.1};
    @Override
    public void runOpMode() throws InterruptedException {
        intake=hardwareMap.get(DcMotor.class,"intake");
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
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
                    x=1;
                }
            }
            if (gamepad1.dpadRightWasPressed()){
                x++;
                if (x>1){
                    x=0;
                }
            }
            if (gamepad1.dpadUpWasPressed()){
                values[x]+=change;
            }
            if (gamepad1.dpadDownWasPressed()){
                values[x]-=change;
            }
            String line1="Values: ";
            for (int i=0;i<=2;i++){
                if (i==x){
                    line1+="{";
                }
                values[i]=(double) Math.round(values[i] * Math.pow(10, 5)) / Math.pow(10, 5);
                line1+=values[i];
                if (i==x){
                    line1+="}";
                }
                line1+=", ";
            }
            if (correctingtoggle){
                spindexer.setPower(values[1]);
                intake.setPower(values[0]);
            }else{
                spindexer.setPower(0);
                intake.setPower(0);
            }
            telemetry.addLine(line1);
            telemetry.update();
        }
    }
}
