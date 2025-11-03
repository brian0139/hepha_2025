package org.firstinspires.ftc.teamcode.Brian;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Alvin.intake;

// TODO: Import your AprilTag and color sensor detection classes
// import org.firstinspires.ftc.teamcode.YourPackage.AprilTagDetector;
// import org.firstinspires.ftc.teamcode.YourPackage.ColorDetector;

public class spindexer {

    private CRServo spindexerServo = null;

    // TODO: Uncomment when ready to integrate
    // private AprilTagDetector aprilTagDetector = null;
    // private ColorDetector colorDetector = null;

    private int[] spindexerSlots = new int[3]; // 0=empty, 1=green, 2=purple
    private int currentPosition = 0;
    private int outtakePosition = 1;

    // Motif patterns: 0=GPP, 1=PGP, 2=PPG
    private int[][] motifPatterns = {
            {1, 2, 2},
            {2, 1, 2},
            {2, 2, 1}
    };
    private int currentMotifPattern = -1;
    private int[] motifPattern;
    private int motifIndex = 0;
    private boolean motifDetected = false;

    private static final double SERVO_SPEED = 0.5;
    private static final long ROTATION_TIME_MS = 500;

    /**
     * Initialize spindexer hardware
     * @param hardwareMap - robot hardware map
     */
    public void init(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, "spindexerServo");
        spindexerServo.setDirection(CRServo.Direction.FORWARD);

        // TODO: Initialize your AprilTag and color sensor detectors here

        java.util.Arrays.fill(spindexerSlots, 0);
    }

    /**
     * Rotate spindexer by one slot position
     */
    public void rotate() {
        spindexerServo.setPower(SERVO_SPEED);
        sleep(ROTATION_TIME_MS);
        spindexerServo.setPower(0);

        currentPosition = (currentPosition + 1) % 3;
        outtakePosition = (outtakePosition + 1) % 3;
    }

    /**
     * Load a ball into current slot
     * @param ballColor - 0=empty, 1=green, 2=purple
     */
    public void loadBall(int ballColor) {
        if (spindexerSlots[currentPosition] == 0) {
            spindexerSlots[currentPosition] = ballColor;
        }
    }

    /**
     * Output ball from outtake position
     * @return true if ball was outputted, false if slot was empty
     */
    public boolean outputBall() {
        if (spindexerSlots[outtakePosition] != 0) {
            spindexerSlots[outtakePosition] = 0;
            motifIndex = (motifIndex + 1) % motifPattern.length;
            return true;
        }
        return false;
    }

    /**
     * Check if the ball at outtake matches required motif pattern
     * @return true if ready to output
     */
    public boolean isReadyToOutput() {
        if (!motifDetected || motifPattern == null) {
            return false;
        }
        int ballAtOuttake = spindexerSlots[outtakePosition];
        int requiredBall = motifPattern[motifIndex];
        return (ballAtOuttake == requiredBall) && (ballAtOuttake != 0);
    }

    /**
     * Automatically rotate to find the required ball for motif pattern
     */
    public void autoRotateToMatch() {
        if (!motifDetected || motifPattern == null) {
            return;
        }

        int requiredBall = motifPattern[motifIndex];
        int rotations = 0;

        while (rotations < 3) {
            if (spindexerSlots[outtakePosition] == requiredBall) {
                break;
            }
            rotate();
            rotations++;
        }
    }

    /**
     * Set motif pattern manually
     * @param patternIndex - 0=GPP, 1=PGP, 2=PPG
     */
    public void setMotifPattern(int patternIndex) {
        if (patternIndex >= 0 && patternIndex <= 2) {
            currentMotifPattern = patternIndex;
            motifPattern = motifPatterns[currentMotifPattern];
            motifIndex = 0;
            motifDetected = true;
        }
    }

    /**
     * TODO: Pull motif pattern from AprilTag detector
     * Should set currentMotifPattern (0-2), motifPattern array, and motifDetected flag
     */
    public void detectMotifFromCamera() {
        // Implement your AprilTag integration here
    }

    /**
     * TODO: Pull ball color from color sensor detector
     * @param intakeSystem - fallback source for ball detection
     * @return 0=empty, 1=green, 2=purple
     */
    public int detectIncomingBall(intake intakeSystem) {
        // Implement your color sensor integration here

        try {
            int[] intakeSlots = intakeSystem.slots;
            if (intakeSlots != null && intakeSlots.length > 0) {
                return intakeSlots[intakeSlots.length - 1];
            }
        } catch (Exception e) {
            // Intake not available
        }

        return 0;
    }

    // Getters

    public int[] getSlots() {
        return spindexerSlots;
    }

    public int getCurrentPosition() {
        return currentPosition;
    }

    public int getOuttakePosition() {
        return outtakePosition;
    }

    public int getRequiredBall() {
        if (motifPattern != null) {
            return motifPattern[motifIndex];
        }
        return 0;
    }

    public int getMotifProgress() {
        return motifIndex;
    }

    public int getMotifLength() {
        if (motifPattern != null) {
            return motifPattern.length;
        }
        return 0;
    }

    public boolean isMotifDetected() {
        return motifDetected;
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
     * Helper method for sleep without requiring LinearOpMode
     * @param milliseconds - time to sleep
     */
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}