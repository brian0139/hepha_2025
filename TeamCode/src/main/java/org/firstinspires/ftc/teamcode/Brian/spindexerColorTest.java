package org.firstinspires.ftc.teamcode.Brian;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Alvin.colorSensor;
import java.util.Vector;

@TeleOp

public class spindexerColorTest extends LinearOpMode{
    CRServo spindexer;
    colorSensor colorsensoroperator;
    spindexerColor spindexercolor;

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        colorsensoroperator=new colorSensor(hardwareMap,"colorSensor");
        spindexercolor=new spindexerColor(spindexer,hardwareMap);
        double spinpower=0.5;
        double stoppower=0.001;
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        Vector<String> detections=new Vector<>();
        //wait for driver to press play
        waitForStart();
        //repeat until opmode ends
        while (opModeIsActive()){
            if (gamepad1.yWasPressed()) spindexercolor.spinToMotif();
            telemetry.addData("motif", spindexercolor.dummyMotif);
            telemetry.addData("next motif index", spindexercolor.dummyMotif[spindexercolor.motifIndex]);
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
            telemetry.update();
        }
    }
}