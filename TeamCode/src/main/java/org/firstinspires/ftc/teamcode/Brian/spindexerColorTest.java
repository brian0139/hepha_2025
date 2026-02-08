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


        spindexercolor=new spindexerColor(spindexer,intake,hardwareMap);

        boolean tmp=false;
        boolean spintointaketoggle=false;
//        FtcDashboard dashboard=FtcDashboard.getInstance();
//        telemetry= dashboard.getTelemetry();
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        Vector<String> detections=new Vector<>();
        //wait for driver to press play
        waitForStart();
        //repeat until opmode ends
        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                motifIndex++;
                motifIndex %= 3;
            }
            if (gamepad1.xWasPressed()){
                spintointaketoggle=!spintointaketoggle;
            }
            if (spintointaketoggle){
                spindexercolor.spinToMotif(motifIndex);
            }else{
                spindexercolor.spindexerServo.setPower(0);
            }
            telemetry.addData("spindexer slots", Arrays.toString(spindexercolor.dummyMotif));
            telemetry.addData("current motif index", motifIndex);
            telemetry.addData("current motif color", spindexercolor.dummyMotif[motifIndex]);
            int detected = colorsensoroperator.getDetected();
//
            String result;
            if (detected == 1) {
                result = "GREEN";
            } else if (detected == 2) {
                result = "PURPLE";
            } else {
                result = "NONE";
            }

            telemetry.addData("Detected", result);


            telemetry.addData("encoderVoltage", spindexerAnalog.getVoltage());
            double posTicks = spindexercolor.getSpindexerTicks();
            telemetry.addData("encoderTicks", posTicks);
            telemetry.addData("Power",spindexercolor.spindexerPID.power);
            telemetry.addData("target voltage",spindexercolor.inslotsV[spindexercolor.currentSlot]);
            telemetry.addData("triggered",
                    Math.abs(spindexercolor.calculateError(spindexercolor.inslotsV[spindexercolor.currentSlot], posTicks)) > spindexercolor.autoSpinEpsilon);
            telemetry.addData("detectioncnt",spindexercolor.detectioncnt);
            telemetry.addData("current slot",spindexercolor.currentSlot);
            telemetry.update();
        }
    }
}
