package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class spindexerTouchTest extends LinearOpMode {
    TouchSensor input=null;
    CRServo servo=null;
    DcMotorEx encoder=null;
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;
    @Override
    public void runOpMode(){
        input=hardwareMap.get(TouchSensor.class,"touch");
        servo=hardwareMap.get(CRServo.class,"spindexerServo");
        encoder=hardwareMap.get(DcMotorEx.class,"intake");
        dashboard=FtcDashboard.getInstance();
        dashboardTelemetry=dashboard.getTelemetry();
        double change=0.1;
        double targetSpeed=0;
        boolean correctingtoggle=false;
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                correctingtoggle = !correctingtoggle;
            }

            // Shift speed
            if (gamepad1.rightBumperWasPressed()) {
                change *= 10;
            } else if (gamepad1.leftBumperWasPressed()) {
                change /= 10;
            }

            telemetry.addData("Change", change);

            if (gamepad1.dpadUpWasPressed()) {
                targetSpeed += change;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetSpeed -= change;
            }

            if (correctingtoggle) {
                servo.setPower(targetSpeed);
            } else {
                servo.setPower(0);
            }

            if (input.isPressed()) {
                correctingtoggle = false;
                servo.setPower(0);
            }


            //encoder telemetry
            telemetry.addData("Encoder Ticks", encoder.getCurrentPosition());
            telemetry.addData("Holding", correctingtoggle);
            telemetry.addData("Target", targetSpeed);
            telemetry.addData("Pressed", input.isPressed());
            telemetry.update();
            if (input.isPressed()) {
                dashboardTelemetry.addData("PressedNum", 1);
            }
            else{
                dashboardTelemetry.addData("PressedNum",0);
            }
            dashboardTelemetry.addData("Pressed",input.isPressed());
            dashboardTelemetry.update();
        }
    }
}