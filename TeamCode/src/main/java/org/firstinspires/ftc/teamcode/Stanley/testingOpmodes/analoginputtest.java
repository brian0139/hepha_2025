package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class analoginputtest extends LinearOpMode {
    AnalogInput input=null;
    CRServo servo=null;
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;
    @Override
    public void runOpMode(){
        input=hardwareMap.get(AnalogInput.class,"spindexerAnalog");
        servo=hardwareMap.get(CRServo.class,"spindexerServo");
        dashboard=FtcDashboard.getInstance();
        dashboardTelemetry=dashboard.getTelemetry();
        waitForStart();
        while(opModeIsActive()){
            servo.setPower(-gamepad1.left_stick_y);
            telemetry.addData("Voltage",input.getVoltage());
            dashboardTelemetry.addData("Voltage",input.getVoltage());
            dashboardTelemetry.update();
            telemetry.update();
        }
    }
}
