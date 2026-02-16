package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class simpleStrafeTest extends LinearOpMode {
    DcMotor leftFront=null;
    DcMotor rightFront=null;
    DcMotor leftBack=null;
    DcMotor rightBack=null;
    @Override
    public void runOpMode(){
        leftFront=hardwareMap.get(DcMotor.class,"leftFront");
        rightFront=hardwareMap.get(DcMotor.class,"rightFront");
        leftBack=hardwareMap.get(DcMotor.class,"leftBack");
        rightBack=hardwareMap.get(DcMotor.class,"rightBack");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        double power=0.2;
        double change=0.1;
        boolean toggle=false;
        waitForStart();
        while (opModeIsActive()){
            //shift speed
            if (gamepad1.rightBumperWasPressed()){
                change*=10;
            }else if (gamepad1.leftBumperWasPressed()){
                change/=10;
            }
            telemetry.addData("Change",change);
            if (gamepad1.dpadUpWasPressed()){
                power+=change;
            }
            if (gamepad1.dpadDownWasPressed()){
                power-=change;
            }
            if (toggle) {
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
            }else{
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
            if (gamepad1.yWasPressed()) toggle=!toggle;
            telemetry.addData("Power",power);
            telemetry.addData("Toggle",toggle);
            telemetry.update();
        }
    }
}
