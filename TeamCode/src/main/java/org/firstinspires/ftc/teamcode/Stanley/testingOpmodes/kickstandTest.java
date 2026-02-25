package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class kickstandTest extends LinearOpMode{
    CRServo servo;

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        servo=hardwareMap.get(CRServo.class,"kickstand");
        waitForStart();
        while (opModeIsActive()){
            servo.setPower(gamepad1.left_stick_y);
        }
    }
}