package org.firstinspires.ftc.teamcode.Brian;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class spindexerCont {
    public CRServo spindexerServo=null;
    public int intakeTouchTimes=0;
    public int outtakeTouchTimes=0;
    public TouchSensor outtakeTouch=null;
    public TouchSensor intakeTouch=null;
    public int currentPosition=0;
    public int numPurple,numGreen;
    public int[] spindexerSlots={0,0,0}; //0 none, 1 green, 2 purple
    public int[][] motifPatterns = {
            {1, 2, 2},
            {2, 1, 2},
            {2, 2, 1}
    };
    public int motifIndex=-1;
    public int[] currentMotifPattern=null;
    public int currentMotifIndex=-1;
    public static final double servoSpeed=0.5;
    public spindexerCont(CRServo spindexerServo){
        this.spindexerServo=spindexerServo;
    }
//work in progress, update real spindexer slots information, 
    public int rotateSpindexerIntake(int reqIntake) {
        if (spindexerSlots[0]!=0&&spindexerSlots[1]!=0&&spindexerSlots[2]!=0){
            return -1;
        }
        while (intakeTouch.isPressed()&&(spindexerSlots[0]==0||spindexerSlots[1]==0||spindexerSlots[2]==0)) {
            spindexerServo.setPower(1);
            if (intakeTouch.isPressed()) {
                intakeTouchTimes++;
                intakeTouchTimes %= 3;
            }
        }
        if(!intakeTouch.isPressed()) {
            currentPosition += intakeTouchTimes;
            return currentPosition;
        } else {
            return -1;
        }
    }
//work in progress
    public int rotateSpindexerOutput(int reqOuttake){
        if (spindexerSlots[0]==0&&spindexerSlots[1]==0&&spindexerSlots[2]==0){
            return -1;
        }
        while (!outtakeTouch.isPressed()&&(spindexerSlots[0]!=0||spindexerSlots[1]!=0||spindexerSlots[2]!=0)) {
            spindexerServo.setPower(1);
            if (!outtakeTouch.isPressed()) {
                outtakeTouchTimes++;
                outtakeTouchTimes %= 3;
            }
        }
        if(outtakeTouch.isPressed()) {
            currentPosition += outtakeTouchTimes;
            return currentPosition;
        } else {
            return -1;
        }
    }
    //work in progress,
    public int rotateSpindexerMotif(){
        while (motifIndex!=currentPosition) {
            if (motifIndex > currentPosition) {
                spindexerServo.setPower(1);
                if (outtakeTouch.isPressed()){
                    outtakeTouchTimes++;
                    outtakeTouchTimes%=3;
                }
            }
        }
        currentPosition+=outtakeTouchTimes;
        if (currentMotifPattern[motifIndex]==spindexerSlots[currentPosition]){
            return currentPosition;
        }
        return -1;
    }


}
