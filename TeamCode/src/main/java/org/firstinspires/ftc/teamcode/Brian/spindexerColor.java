package org.firstinspires.ftc.teamcode.Brian;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Alvin.colorSensor;

public class spindexerColor {
    public CRServo spindexerServo=null;

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
    public int[] currentMotifPattern=null;
    public static final double servoSpeed=0.5;
    public spindexerColor(CRServo spindexerServo){
        this.spindexerServo=spindexerServo;
    }

    public boolean spinToMotif(double power) {
        int nextmotif=currentMotifPattern[motifIndex];
        int timeout=0;
        while (colorsensor.getDetected()!=nextmotif&&timeout<3) {
            spindexerServo.setPower(0.5);
            timeout++;
        }
        if (colorsensor.getDetected()==nextmotif&&timeout<3){
            motifIndex++;
            return true;
        } else {
            return false;
        }
    }
}
