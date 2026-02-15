package org.firstinspires.ftc.teamcode.Stanley.tuningStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

@TeleOp
public class tuningFlywheel extends LinearOpMode {
    DcMotorEx flywheel=null;
    DcMotorEx flywheelR=null;
    DcMotor transfer=null;
    DcMotor intake=null;
    CRServo spindexer=null;
    double change=0.1;
    int x=0;
    boolean correctingtoggle=false;
    boolean transferToggle=false;
    int targetSpeed=2100;
    //FTC dashboard telemetry
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;
    PIDFCoefficients flywheelCoefficients=new PIDFCoefficients(0,0,0,0);

    @Override
    public void runOpMode(){
        //Formatting
        DecimalFormat df=new DecimalFormat("0.0#");
        df.setMaximumFractionDigits(340);
        flywheel=hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelR=hardwareMap.get(DcMotorEx.class,"flywheelR");
        flywheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake=hardwareMap.get(DcMotor.class,"intake");
        transfer=hardwareMap.get(DcMotor.class,"par1");
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        double[] Kf={0,0,0,0};
        //initialize PIDF values
        flywheelCoefficients=flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        Kf[0]=flywheelCoefficients.p;
        Kf[1]=flywheelCoefficients.i;
        Kf[2]=flywheelCoefficients.d;
        Kf[3]=flywheelCoefficients.f;
        waitForStart();
        while (opModeIsActive()){
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            spindexer.setPower(-gamepad1.right_stick_x);
            targetSpeed+=(int)-gamepad1.left_stick_y*3;
            if (gamepad1.yWasPressed()){
                correctingtoggle=!correctingtoggle;
            }
            //shift speed
            if (gamepad1.rightBumperWasPressed()){
                change*=10;
            }else if (gamepad1.leftBumperWasPressed()){
                change/=10;
            }
            telemetry.addData("Change",df.format(change));
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
                Kf[i]=(double) Math.round(Kf[i] * Math.pow(10, 5)) / Math.pow(10, 5);
                line1+=Kf[i];
                if (i==x){
                    line1+="}";
                }
                line1+=", ";
            }
            if (gamepad1.xWasPressed()){
                flywheelCoefficients=new PIDFCoefficients(Kf[0],Kf[1],Kf[2],Kf[3]);
                flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,flywheelCoefficients);
                flywheelR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,flywheelCoefficients);
            }
            if (gamepad1.aWasPressed()) transferToggle=!transferToggle;
            if (transferToggle){
                transfer.setPower(-1);
            }else{
                transfer.setPower(0);
            }
            if (correctingtoggle){
                flywheel.setVelocity(-targetSpeed);
                flywheelR.setVelocity(-targetSpeed);
            }else{
                flywheel.setVelocity(0);
                flywheelR.setVelocity(0);
            }
            telemetry.addLine(line1);
            telemetry.addData("Holding",correctingtoggle);
            telemetry.addData("Target",targetSpeed);
            telemetry.addData("CurrentR",-flywheelR.getVelocity());
            telemetry.addData("CurrentL",-flywheel.getVelocity());
            telemetry.addData("Offset(ticks)",targetSpeed+flywheel.getVelocity());
            telemetry.addData("OffsetR(ticks)",targetSpeed+flywheelR.getVelocity());
            telemetry.update();
            dashboardTelemetry.addLine(line1);
            dashboardTelemetry.addData("Holding",correctingtoggle);
            dashboardTelemetry.addData("Target",targetSpeed);
            dashboardTelemetry.addData("CurrentR",-flywheelR.getVelocity());
            dashboardTelemetry.addData("CurrentL",-flywheel.getVelocity());
            dashboardTelemetry.addData("Offset(ticks)",targetSpeed+flywheel.getVelocity());
            dashboardTelemetry.addData("OffsetR(ticks)",targetSpeed+flywheelR.getVelocity());
            dashboardTelemetry.update();
        }
    }
}
