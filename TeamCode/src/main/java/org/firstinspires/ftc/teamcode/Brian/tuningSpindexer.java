package org.firstinspires.ftc.teamcode.Brian;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.PID;

@TeleOp
public class tuningSpindexer extends LinearOpMode {
    CRServo spindexer=null;
    AnalogInput spindexerAnalog = null;
    spindexerColor spindexerOperator=null;
    double change=0.1;
    int x=0;
    //TODO:Get real value+sync with outtakeV2 value
    //test
    double targetVoltage = 0.131;
    boolean correctingtoggle=false;
    //FTC dashboard telemetry
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;

    @Override
    public void runOpMode(){
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        spindexerAnalog = hardwareMap.get(AnalogInput.class, "spindexerAnalog");
        spindexerOperator=new spindexerColor(spindexer,null,hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.yWasPressed()) correctingtoggle=!correctingtoggle;
            targetVoltage += -gamepad1.left_stick_y * 0.05;
            if (gamepad1.rightStickButtonWasPressed()){
                spindexerColor.ENCODER_ZERO_TICKS = spindexerOperator.getSpindexerTicks();
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
            if (targetVoltage>spindexerColor.ENCODER_MAX_VOLTAGE) targetVoltage-=spindexerColor.ENCODER_MAX_VOLTAGE;
            if (targetVoltage<0) targetVoltage+=spindexerColor.ENCODER_MAX_VOLTAGE;
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
                double targetTicks = (targetVoltage / spindexerColor.ENCODER_MAX_VOLTAGE) * spindexerColor.ENCODER_TICKS_PER_REV;
                double currentTicks = spindexerOperator.getSpindexerTicks();
                spindexerOperator.spindexerServo.setPower(
                        spindexerOperator.spindexerPID.update(spindexerOperator.calculateError(targetTicks, currentTicks)));
            }else{
                spindexerOperator.spindexerServo.setPower(0);
            }
//            spindexerOperator.updateHoodAngle();

            telemetry.addLine(line1);
            telemetry.addData("Holding",correctingtoggle);
            telemetry.addData("TargetVoltage",targetVoltage);
            telemetry.addData("TargetTicks",(targetVoltage / spindexerColor.ENCODER_MAX_VOLTAGE) * spindexerColor.ENCODER_TICKS_PER_REV);
//            telemetry.addData("Current",spindexerOperator.hoodAngle*spindexerOperator.servoDegPerRot);
            telemetry.addData("EncoderVoltage",spindexerAnalog.getVoltage());
            telemetry.addData("CurrentTicks",spindexerOperator.getSpindexerTicks());
            telemetry.addData("Error",spindexerOperator.calculateError(
                    (targetVoltage / spindexerColor.ENCODER_MAX_VOLTAGE) * spindexerColor.ENCODER_TICKS_PER_REV,
                    spindexerOperator.getSpindexerTicks()));
            telemetry.addData("Power",spindexerOperator.spindexerPID.power);
//            telemetry.addData("Offset(rotations)",angle/spindexerOperator.servoDegPerRot-spindexerOperator.hoodAngle);
//            telemetry.addData("AtTarget",atTarget);
            telemetry.update();
            dashboardTelemetry.addLine(line1);
            dashboardTelemetry.addData("Holding",correctingtoggle);
            dashboardTelemetry.addData("TargetVoltage",targetVoltage);
            dashboardTelemetry.addData("TargetTicks",(targetVoltage / spindexerColor.ENCODER_MAX_VOLTAGE) * spindexerColor.ENCODER_TICKS_PER_REV);
//            dashboardTelemetry.addData("Current",spindexerOperator.hoodAngle);
            dashboardTelemetry.addData("EncoderVoltage",spindexerAnalog.getVoltage());
            dashboardTelemetry.addData("CurrentTicks",spindexerOperator.getSpindexerTicks());
            dashboardTelemetry.addData("Error",spindexerOperator.calculateError(
                    (targetVoltage / spindexerColor.ENCODER_MAX_VOLTAGE) * spindexerColor.ENCODER_TICKS_PER_REV,
                    spindexerOperator.getSpindexerTicks()));
            dashboardTelemetry.addData("Power",spindexerOperator.spindexerPID.power);
//            dashboardTelemetry.addData("Offset(rotations)",angle/spindexerOperator.servoDegPerRot-spindexerOperator.hoodAngle);
//            dashboardTelemetry.addData("AtTarget",atTarget);
            dashboardTelemetry.update();
        }
    }
}
