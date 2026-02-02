package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.PIDF;

@TeleOp
public class tuningFlywheel extends LinearOpMode {
    DcMotorEx flywheel=null;
    double change=0.1;
    int x=0;
    boolean correctingtoggle=false;
    int targetSpeed=2100;
    //FTC dashboard telemetry
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;
    PIDF flywheelPIDF =new PIDF(0,0,0,0);

    @Override
    public void runOpMode(){
        flywheel=hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        double[] Kf={0,0,0,0};
        waitForStart();
        while (opModeIsActive()){
            targetSpeed+=(int)gamepad1.left_stick_y;
            if (gamepad1.yWasPressed()){
                correctingtoggle=!correctingtoggle;
            }
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
                    x=3;
                }
            }
            if (gamepad1.dpadRightWasPressed()){
                x++;
                if (x>3){
                    x=0;
                }
            }
            if (gamepad1.dpadUpWasPressed()){
                Kf[x]+=change;
            }
            if (gamepad1.dpadDownWasPressed()){
                Kf[x]-=change;
            }
            String line1="Parameters: ";
            for (int i=0;i<=3;i++){
                if (i==x){
                    line1+="{";
                }
                Kf[i]=(double) Math.round(Kf[i] * Math.pow(10, 20)) / Math.pow(10, 20);
                line1+=Kf[i];
                if (i==x){
                    line1+="}";
                }
                line1+=", ";
            }
            if (gamepad1.xWasPressed()){
                flywheelPIDF =new PIDF(Kf[0],Kf[1],Kf[2],Kf[3]);
            }
            boolean atTarget=false;
            if (correctingtoggle){
                flywheel.setPower(flywheelPIDF.update(targetSpeed,flywheel.getVelocity()));
            }else{
                flywheel.setPower(0);
            }
            telemetry.addLine(line1);
            telemetry.addData("Holding",correctingtoggle);
            telemetry.addData("Target",targetSpeed);
            telemetry.addData("Current",flywheel.getVelocity());
            telemetry.addData("Power",flywheelPIDF.power);
            telemetry.addData("P",flywheelPIDF.P);
            telemetry.addData("I",flywheelPIDF.I);
            telemetry.addData("D",flywheelPIDF.D);
            telemetry.addData("F",flywheelPIDF.F);
            telemetry.addData("Offset(ticks)",targetSpeed-flywheel.getVelocity());
            telemetry.addData("AtTarget",atTarget);
            telemetry.update();
            dashboardTelemetry.addLine(line1);
            dashboardTelemetry.addData("Holding",correctingtoggle);
            dashboardTelemetry.addData("Target",targetSpeed);
            dashboardTelemetry.addData("Current",flywheel.getVelocity());
            dashboardTelemetry.addData("Power",flywheelPIDF.power);
            dashboardTelemetry.addData("P",flywheelPIDF.P);
            dashboardTelemetry.addData("I",flywheelPIDF.I);
            dashboardTelemetry.addData("D",flywheelPIDF.D);
            dashboardTelemetry.addData("F",flywheelPIDF.F);
            dashboardTelemetry.addData("Offset(ticks)",targetSpeed-flywheel.getVelocity());
            dashboardTelemetry.addData("AtTarget",atTarget);
            dashboardTelemetry.update();
        }
    }
}
