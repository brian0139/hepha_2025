package org.firstinspires.ftc.teamcode.Brian;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alvin.colorSensor;

public class spindexerColor {
    public CRServo spindexerServo=null;
    ElapsedTime timer=new ElapsedTime();
    colorSensor colorsensor;
    public int currentPosition=0;
    public int numPurple,numGreen;
    public int[] spindexerSlots={0,0,0}; //0 none, 1 green, 2 purple
    public int[][] motifPatterns = {
            {1, 2, 2},
            {2, 1, 2},
            {2, 2, 1}
    };
    public int motifIndex=0;
    public int[] dummyMotif={1,2,2};
    public int[] currentMotifPattern=null;
    public boolean successfulrotate=false;
    public static final double servoSpeed=0.5;
    public spindexerColor(CRServo spindexerServo, HardwareMap hardwareMap){
        this.spindexerServo=spindexerServo;
        colorsensor=new colorSensor(hardwareMap,"colorSensor");
    }

    public boolean spinToMotif() {
        int nextmotif=dummyMotif[motifIndex];
        int timeout=0;
        while (colorsensor.getDetected()!=nextmotif&&timeout<3) {
            spindexerServo.setPower(0.5);
            timeout++;
        }
        if (colorsensor.getDetected()==nextmotif&&timeout<3){
            motifIndex++;
            timer.reset();
            spindexerServo.setPower(-0.01);
            if (timer.milliseconds()>=400){
                spindexerServo.setPower(0);
            }
            successfulrotate=true;
            return true;
        } else {
            timer.reset();
            spindexerServo.setPower(-0.01);
            if (timer.milliseconds()>=400){
                spindexerServo.setPower(0);
            }
            successfulrotate=false;
            return false;
        }
    }
}
