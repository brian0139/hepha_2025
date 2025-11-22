package org.firstinspires.ftc.teamcode.Brian;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Aaron.aprilTag;
import org.firstinspires.ftc.teamcode.Alvin.intake;

// TODO: Import your AprilTag and color sensor detection classes
// import org.firstinspires.ftc.teamcode.YourPackage.AprilTagDetector;
// import org.firstinspires.ftc.teamcode.YourPackage.ColorDetector;

@TeleOp(name="spindexer", group="FTC")
public class spindexer{

    public Servo spindexerServo = null;

    public int[] spindexerSlots = new int[3]; // 0=empty, 1=green, 2=purple
    public int currentPosition = 0;
    public int outtakePosition = 1;

    // Motif patterns: 0=GPP, 1=PGP, 2=PPG
    public int[][] motifPatterns = {
            {1, 2, 2},
            {2, 1, 2},
            {2, 2, 1}
    };
    public int currentMotifPattern = -1;
    public int[] motifPattern;
    public int motifIndex = 0;
    public boolean motifDetected = false;

    public static final double SERVO_SPEED = 0.5;
    public static final long ROTATION_TIME_MS = 500;
    public double currentintake, currentouttake;

    // Intake slots (servo positions)
    public double[] intakeslots = {0.0/360, 120.0/360, 240.0/360};
    // Outtake slots (servo positions)
    public double[] outtakeslots = {60.0/360, 180.0/360, 300.0/360};

    public void rotateSpindexerInput() {
        int reqIntake = 0;
        spindexerServo.setPosition(intakeslots[reqIntake]);
    }

    public void rotateSpindexerOutput() {
        int reqOuttake = 0; // TODO: Calculate required outtake position
        spindexerServo.setPosition(outtakeslots[reqOuttake]);
    }

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
     * @param pattern - array of ball colors in pattern
     * @return formatted string with current position marked
     */
    public String getPatternString(int[] pattern) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < pattern.length; i++) {
            if (i == motifIndex) {
                sb.append("[").append(getBallName(pattern[i])).append("]");
            } else {
                sb.append(getBallName(pattern[i]));
            }
            if (i < pattern.length - 1) {
                sb.append(", ");
            }
        }
        return sb.toString();
    }

    /**
     * Searches all slots to find the required ball for current motif position
     */
    public void autoRotateToMatchMotif() {
        int requiredBall = motifPattern[motifIndex];
        int rotations = 0;

        while (rotations < 3) {
            if (spindexerSlots[outtakePosition] == requiredBall) {
                break;
            }
            rotateSpindexerInput();
            rotations++;
        }
    }
}