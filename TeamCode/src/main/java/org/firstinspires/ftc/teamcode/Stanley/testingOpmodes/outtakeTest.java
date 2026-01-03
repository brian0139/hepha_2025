package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class outtakeTest extends LinearOpMode{
    private CRServo hoodServo=null;
    DcMotorEx flywheelDrive=null;
    DcMotorEx flywheelDriveR=null;
    DcMotor transfer=null;
    boolean toggle=false;


    //main loop
    @Override
    public void runOpMode() {
        //outtake instance
        hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        AnalogInput hoodSensor=hardwareMap.get(AnalogInput.class,"hoodAnalog");
        flywheelDrive=hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheelDriveR=hardwareMap.get(DcMotorEx.class,"flywheelR");
        transfer=hardwareMap.get(DcMotor.class,"par1");
        FtcDashboard dashboard=FtcDashboard.getInstance();
        Telemetry dashboardtelemetry=dashboard.getTelemetry();
        outtakeV3 outtakeOperator = new outtakeV3(hardwareMap, "Red",false);
        outtakeOperator.initHoodAngleBlocking();

        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.yWasPressed()) outtakeOperator.initHoodAngleBlocking();
            if (gamepad1.aWasPressed()) toggle=!toggle;
            if (toggle){
                outtakeOperator.setHood(40.28);
            }else{
                outtakeOperator.setHood(66.81);
            }
            telemetry.update();
        }
    }
}