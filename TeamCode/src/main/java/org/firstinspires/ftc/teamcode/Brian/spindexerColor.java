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
    ElapsedTime nonetime=new ElapsedTime();
    public double nonetimer=0;
    public int timeout=0;
    colorSensor outtakesensor;
    colorSensor intakesensor;
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
    public static final double servoSpeed=0.5;
    public spindexerColor(CRServo spindexerServo, HardwareMap hardwareMap){
        this.spindexerServo=spindexerServo;
        outtakesensor=new colorSensor(hardwareMap,"outtakeSensor");
        intakesensor=new colorSensor(hardwareMap, "intakeSensor");
    }

    public boolean spinToMotif() {
        int nextmotif=dummyMotif[motifIndex];
        timeout=0;
        boolean detectedLastLoop=false;
        while (outtakesensor.getDetected()!=nextmotif&&timeout<3) {
            spindexerServo.setPower(0.75);
            if ((outtakesensor.getDetected()==1||outtakesensor.getDetected()==2) && !detectedLastLoop){
                timeout++;
                detectedLastLoop=true;
            }else{
                detectedLastLoop=false;
            }
        }
        if (outtakesensor.getDetected()==nextmotif&&timeout<3){
            motifIndex++;
            motifIndex%=3;
            timer.reset();
            spindexerServo.setPower(-0.01);
            if (timer.milliseconds()>=400){
                spindexerServo.setPower(0);
            }
            return true;
        } else {
            timer.reset();
            spindexerServo.setPower(-0.01);
            if (timer.milliseconds()>=400){
                spindexerServo.setPower(0);
            }
            return false;
        }
    }
    public boolean spinToIntake(){
        int nextMotif=dummyMotif[motifIndex];
        timeout=0;
        boolean noneTrue=false;
        boolean detectedLastLoop=true;  // Start as true (assume we start on filled)
        nonetime.reset();
        ElapsedTime emptyTimer = new ElapsedTime();
        boolean seenEmpty = false;

        while (timeout<3 && nonetime.milliseconds()<2000){
            int currentReading = intakesensor.getDetected();

            // Count when transitioning from filled (1 or 2) to empty (0)
            if (currentReading==0 && detectedLastLoop==true) {
                timeout++;
                detectedLastLoop = false;
            } else if (currentReading!=0) {
                detectedLastLoop=true;
            }

            // Slow down when we see empty readings
            if (currentReading==0) {
                if (!seenEmpty) {
                    emptyTimer.reset();
                    seenEmpty = true;
                }

                // Wait 300ms to confirm it's a real empty slot, not a gap
                if (emptyTimer.milliseconds() >= 300) {
                    spindexerServo.setPower(0.1);  // Very slow for final positioning
                    if (emptyTimer.milliseconds() >= 400) {
                        break;  // Definitely a real empty slot
                    }
                } else {
                    spindexerServo.setPower(0.4);  // Continue at normal speed
                }
            } else {
                spindexerServo.setPower(0.4);  // Normal speed on filled slots
                seenEmpty = false;
            }
        }

        if (intakesensor.getDetected()==0&&timeout<3){
            for (int i=0; i<3;i++){
                if (spindexerSlots[i]!=0){
                    spindexerSlots[i]=intakesensor.getDetected();
                }
            }
            timer.reset();
            spindexerServo.setPower(-0.01);
            if (timer.milliseconds()>=400){
                spindexerServo.setPower(0);
            }
            return true;
        }else{
            timer.reset();
            spindexerServo.setPower(-0.01);
            if (timer.milliseconds()>=400){
                spindexerServo.setPower(0);
            }
            return false;
        }
    }
}
