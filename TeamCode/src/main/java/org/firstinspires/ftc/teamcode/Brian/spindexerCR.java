package org.firstinspires.ftc.teamcode.Brian;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Aaron.aprilTag;
import org.firstinspires.ftc.teamcode.Alvin.colorSensor;
import org.firstinspires.ftc.teamcode.Alvin.intake;


public class spindexerCR{

    public Servo spindexerServo = null;
    public int touchSensorTaps=0;

    public int[] spindexerSlots = {2,2,1}; // 0=empty, 1=green, 2=purple
    public int currentPosition = 0;
    public int outtakePosition = 0;
    public int intakePosition = 0;

    // Motif patterns: 1:green, 2: purple
    public int[][] motifPatterns = {
            {1, 2, 2},
            {2, 1, 2},
            {2, 2, 1}
    };
    public int currentMotifPattern = -1;
    public int[] motifPattern = null; // Will be set from motifPatterns when AprilTag is detected
    public int motifIndex = 0; //where in the motif you are

    public static final double SERVO_SPEED = 0.5;
    public static final long ROTATION_TIME_MS = 500;
    public double currentintake, currentouttake;
    public int numPurple, numGreen;
    int emptySlot;

    public double[] spindexerPos = {0,0.75};
    public int currentSpindexerPos=1; //0.75 is initial

    public spindexerCR(Servo spindexerServo){
        this.spindexerServo=spindexerServo;
    }

        public int rotateSpindexer() {
            if (spindexerServo.getPosition()>spindexerPos[currentSpindexerPos]){
                spindexerServo.setPosition(spindexerServo.getPosition()-0.005);
            } else if (spindexerServo.getPosition()<spindexerPos[currentSpindexerPos]){
                spindexerServo.setPosition(spindexerPos[currentSpindexerPos]);
            }

            return currentSpindexerPos;
        }


    // calling intake if intakeuntilpixel until returns true
    // if returns true then read color of ball and decide if want to spit it out or intake
//    public int detectIncomingBall(intake intakeSystem, colorSensor colorSensor) {
//        if (intakeSystem.intakeUntilPixel(1000)) {
//            int detectedColor = colorSensor.getDetected();
//            for (int i = 0; i < 3; i++) {
//                if (spindexerSlots[i] == 1) numGreen++;
//                else if (spindexerSlots[i] == 2) numPurple++;
//                else emptySlot = i;
//            }
//            boolean reject = false;
//
//            if ((detectedColor == 2 && numPurple >= 2) || (detectedColor == 1 && numGreen >= 2) && (emptySlot == 0 || emptySlot == 1 || emptySlot == 2)) {
//                rotateSpindexerInput(emptySlot);
//                intakeSystem.intake();
//                spindexerSlots[emptySlot] = colorSensor.getDetected(); // update spindexer slot to color that we intake
//                return 0; // intake ball succesfully
//            } else {
//                intakeSystem.reverse(); // spit out ball if color is not wanted
//                return -1; // spit out ball
//            }
//        }
//        return -1;
//    }


//    public void spinToOuttake() {
//        if (motifPattern == null) return; // No motif pattern set yet
//        if (motifIndex < 0 || motifIndex >= motifPattern.length) return; // Bounds check
//        int requiredBall = motifPattern[motifIndex];
//        for (int i = 0; i < 3; i++) {
//            if (spindexerSlots[i] != 0) { // check if slot is empty or not
//                if (spindexerSlots[i] == requiredBall) {
//                    rotateSpindexerOutput(i);
//                }
//            }
//        }
//    }

    /**
     * Increments motifIndex after a ball is outtaken.
     * Resets to 0 if it reaches the end of the pattern.
     */
    public void incrementMotifIndex() {
        if (motifPattern != null && motifIndex < motifPattern.length - 1) {
            motifIndex++;
        } else {
            motifIndex = 0; // Reset to start of pattern
        }
    }

    /**
     * Resets motifIndex to 0 (start of pattern)
     */
    public void resetMotifIndex() {
        motifIndex = 0;
    }

    /**
     * @param ballType - 0=empty, 1=green, 2=purple
     * @return human-readable ball color name
     */
    public String getBallName(int ballType) {
        switch (ballType) {
            case 0: return "Empty";
            case 1: return "Green";
            case 2: return "Purple";
            default: return "Unknown";
        }
    }

    /**
     * @param patternIndex - 0=GPP, 1=PGP, 2=PPG
     * @return human-readable pattern name
     */
    public String getMotifName(int patternIndex) {
        switch (patternIndex) {
            case 0: return "GPP (Green-Purple-Purple)";
            case 1: return "PGP (Purple-Green-Purple)";
            case 2: return "PPG (Purple-Purple-Green)";
            default: return "Unknown";
        }
    }


    /**
     * Searches all slots to find the required ball for current motif position
     */
//    public void autoRotateToMatchMotif() {
//        if (motifPattern == null) return; // No motif pattern set yet
//        if (motifIndex < 0 || motifIndex >= motifPattern.length) return; // Bounds check
//        int requiredBall = motifPattern[motifIndex];
//
//        for (int i = 0; i < 3; i++) {
//            if (spindexerSlots[i] == requiredBall) {
//                rotateSpindexerInput(i);
//                break;
//            }
//        }
//
//    }
}