package org.firstinspires.ftc.teamcode.Brian;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Alvin.colorSensor;
import java.util.Vector;
import java.util.*;

@TeleOp

public class spindexerColorTest extends LinearOpMode{
    CRServo spindexer;
    colorSensor colorsensoroperator;
    spindexerColor spindexercolor;
    AnalogInput spindexerAnalog;
    DcMotor intake;
    int motifIndex=0;

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        colorsensoroperator=new colorSensor(hardwareMap,"outtakeSensor");
        spindexerAnalog=hardwareMap.get(AnalogInput.class,"spindexerAnalog");
        intake=hardwareMap.get(DcMotor.class, "intake");


        spindexercolor=new spindexerColor(spindexer,hardwareMap);

        boolean tmp=false;
//        FtcDashboard dashboard=FtcDashboard.getInstance();
//        telemetry= dashboard.getTelemetry();
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        Vector<String> detections=new Vector<>();
        //wait for driver to press play
        waitForStart();
        //repeat until opmode ends
        while (opModeIsActive()){
            if (gamepad1.yWasPressed()) {
                spindexercolor.spinToMotif(motifIndex);
                motifIndex++;
                motifIndex%=3;
            }

//            intake.setPower(0.75);
//            if (gamepad1.yWasPressed()) spindexercolor.spinToMotif();
//            if (gamepad1.yWasPressed()) spindexercolor.spinToIntake();
            telemetry.addData("spindexer slots", Arrays.toString(spindexercolor.dummyMotif));
//            telemetry.addData("time elapsed", spindexercolor.nonetimer);
//            telemetry.addData("timeout",spindexercolor.timeout);
            telemetry.addData("current motif index", motifIndex);
            telemetry.addData("current motif color", spindexercolor.dummyMotif[motifIndex]);
            int detected = colorsensoroperator.getDetected();

            String result;
            if (detected == 1) {
                result = "GREEN";
            } else if (detected == 2) {
                result = "PURPLE";
            } else {
                result = "NONE";
            }

            telemetry.addData("Detected", result);

            float[] hsv = colorsensoroperator.readHSV();
            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Sat", hsv[1]);
            telemetry.addData("Val", hsv[2]);

            telemetry.addData("spindexer power",0.75);
            telemetry.addData("stopping power",-0.01);
            telemetry.addData("tmp boolean", tmp);
            telemetry.update();
        }
    }
}