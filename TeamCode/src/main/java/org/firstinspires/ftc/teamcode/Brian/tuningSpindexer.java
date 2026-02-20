package org.firstinspires.ftc.teamcode.Brian;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.PID;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.PIDspindexer;

@TeleOp
public class tuningSpindexer extends LinearOpMode {
    CRServo spindexer=null;
    DcMotorEx spindexerSensor =null;
    spindexerColor spindexerOperator=null;
    double change=0.1;
    int x=0;
    //TODO:Get real value+sync with outtakeV2 value
    //test
    double angle=0.131;
    boolean correctingtoggle=false;
    //FTC dashboard telemetry
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;

    @Override
    public void runOpMode(){
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        spindexerSensor =hardwareMap.get(DcMotorEx.class,"intake");
        spindexerOperator=new spindexerColor(spindexer,null,hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.yWasPressed()) correctingtoggle=!correctingtoggle;
            angle+=-gamepad1.left_stick_y*5;
            if (gamepad1.rightStickButtonWasPressed()){
                spindexerSensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spindexerSensor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                spindexerOperator.kS[x]+=change;
            }
            if (gamepad1.dpadDownWasPressed()){
                spindexerOperator.kS[x]-=change;
            }
            if (angle>3.3) angle-=3.3;
            if (angle<0) angle+=3.3;
            String line1="Kh: ";
            for (int i=0;i<=2;i++){
                if (i==x){
                    line1+="{";
                }
                spindexerOperator.kS[i]=(double) Math.round(spindexerOperator.kS[i] * Math.pow(10, 7)) / Math.pow(10, 7);
                line1+=spindexerOperator.kS[i];
                if (i==x){
                    line1+="}";
                }
                line1+=", ";
            }
            if (gamepad1.xWasPressed()){
                spindexerOperator.spindexerPID=new PID(spindexerOperator.kS[0],spindexerOperator.kS[1],spindexerOperator.kS[2]);
            }
//            boolean atTarget=false;
            if (correctingtoggle){
                spindexerOperator.spindexerServo.setPower(spindexerOperator.spindexerPID.update(spindexerOperator.calculateError(angle,spindexerOperator.spindexerSensor.getCurrentPosition())));
            }else{
                spindexerOperator.spindexerServo.setPower(0);
            }
//            spindexerOperator.updateHoodAngle();

            telemetry.addLine(line1);
            telemetry.addData("Holding",correctingtoggle);
            telemetry.addData("Target",angle);
//            telemetry.addData("Current",spindexerOperator.hoodAngle*spindexerOperator.servoDegPerRot);
            telemetry.addData("CurrentEncoder",spindexerOperator.spindexerSensor.getCurrentPosition()%8192);
            telemetry.addData("Error",spindexerOperator.calculateError(angle,spindexerOperator.spindexerSensor.getCurrentPosition()));
            telemetry.addData("Power",spindexerOperator.spindexerPID.power);
//            telemetry.addData("Offset(rotations)",angle/spindexerOperator.servoDegPerRot-spindexerOperator.hoodAngle);
//            telemetry.addData("AtTarget",atTarget);
            telemetry.update();
            dashboardTelemetry.addLine(line1);
            dashboardTelemetry.addData("Holding",correctingtoggle);
            dashboardTelemetry.addData("Target",angle);
//            dashboardTelemetry.addData("Current",spindexerOperator.hoodAngle);
            dashboardTelemetry.addData("CurrentEncoder",spindexerOperator.spindexerSensor.getCurrentPosition()%8192);
            dashboardTelemetry.addData("Error",spindexerOperator.calculateError(angle,spindexerOperator.spindexerSensor.getCurrentPosition()));
            dashboardTelemetry.addData("Power",spindexerOperator.spindexerPID.power);
//            dashboardTelemetry.addData("Offset(rotations)",angle/spindexerOperator.servoDegPerRot-spindexerOperator.hoodAngle);
//            dashboardTelemetry.addData("AtTarget",atTarget);
            dashboardTelemetry.update();
        }
    }
}
