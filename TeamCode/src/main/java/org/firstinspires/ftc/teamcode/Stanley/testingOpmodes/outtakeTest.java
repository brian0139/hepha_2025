package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtake;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        double servoRPM=50;
        double msPerRotation=60000/servoRPM;
        double hoodAngle=30;
        //outtake instance
        hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        flywheelDrive=hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheelDriveR=hardwareMap.get(DcMotorEx.class,"flywheelR");
        transfer=hardwareMap.get(DcMotor.class,"par1");
        ElapsedTime timer=new ElapsedTime();
        //TODO:add hood Sensor
        outtakeV2 outtakeOperator = new outtakeV2(hardwareMap, flywheelDrive, flywheelDriveR,"Red",null,null,null,null,hoodServo,null,transfer,false);
        outtakeOperator.hoodAngle=hoodAngle;
        outtakeOperator.savehoodAngle=hoodAngle;

        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        while (opModeIsActive()){
            hoodAngle-=gamepad1.left_stick_y*0.001;
            hoodServo.setPower(gamepad1.right_stick_y);
            if (gamepad1.rightBumperWasPressed()){
                timer.reset();
                while (timer.milliseconds()<msPerRotation) hoodServo.setPower(1);
                hoodServo.setPower(0);
            }
            if (gamepad1.leftBumperWasPressed()){
                timer.reset();
                while (timer.milliseconds()<msPerRotation) hoodServo.setPower(-1);
                hoodServo.setPower(0);
            }
            if (gamepad1.yWasPressed()){
                outtakeOperator.setHood(hoodAngle);
            }
            telemetry.addData("Running hood",outtakeOperator.runninghood);
            telemetry.addData("targethoodangle",hoodAngle);
            if (outtakeOperator.runninghood) {
                outtakeOperator.updateHoodAngle(hoodAngle);
                telemetry.addData("Expected hood angle", outtakeOperator.hoodAngle);
            }
            telemetry.update();
        }
    }
}