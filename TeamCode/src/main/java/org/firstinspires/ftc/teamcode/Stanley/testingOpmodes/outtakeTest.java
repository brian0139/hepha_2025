package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;

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


    //main loop
    @Override
    public void runOpMode() {
        //outtake instance
        hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        AnalogInput hoodSensor=hardwareMap.get(AnalogInput.class,"hoodAnalog");
        flywheelDrive=hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheelDriveR=hardwareMap.get(DcMotorEx.class,"flywheelR");
        transfer=hardwareMap.get(DcMotor.class,"par1");
        ElapsedTime timer=new ElapsedTime();
        //TODO:add hood Sensor
        outtakeV2 outtakeOperator = new outtakeV2(hardwareMap, flywheelDrive, flywheelDriveR,"Red",null,null,null,null,hoodServo,hoodSensor,transfer,false);

        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.rightBumperWasPressed()) {
                while(hoodSensor.getVoltage()<=3.28) hoodServo.setPower(0.2);
            }
            if (gamepad1.leftBumperWasPressed()) {
                while(hoodSensor.getVoltage()>=0.2) hoodServo.setPower(-0.2);
            }
            telemetry.update();
        }
    }
}