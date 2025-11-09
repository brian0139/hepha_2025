package org.firstinspires.ftc.teamcode.Brian;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Alvin.intake;

// TODO: Import your AprilTag and color sensor detection classes
// import org.firstinspires.ftc.teamcode.YourPackage.AprilTagDetector;
// import org.firstinspires.ftc.teamcode.YourPackage.ColorDetector;

@TeleOp(name="spindexer", group="FTC")
public class spindexer extends LinearOpMode {

    private Servo spindexerServo = null;

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

    @Override
    public void runOpMode() {
        spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        spindexerServo.setDirection(Servo.Direction.FORWARD);

        // TODO: Initialize your AprilTag and color sensor detectors here

        java.util.Arrays.fill(spindexerSlots, 0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        detectMotifFromCamera();
        intake intakeSystem = new intake();

        while (opModeIsActive()) {
            if (!motifDetected) {
                detectMotifFromCamera();
            }

            if (!motifDetected || motifPattern == null) {
                telemetry.addData("Status", "Waiting for motif");
                telemetry.update();

                if (gamepad1.x) {
                    detectMotifFromCamera();
                    sleep(200);
                }

                sleep(100);
                continue;
            }

            int incomingBall = detectIncomingBall(intakeSystem);

            if (incomingBall > 0 && spindexerSlots[currentPosition] == 0) {
                spindexerSlots[currentPosition] = incomingBall;
            }

            int ballAtOuttake = spindexerSlots[outtakePosition];
            int requiredBall = motifPattern[motifIndex];
            boolean shouldOutput = (ballAtOuttake == requiredBall) && (ballAtOuttake != 0);

            if (shouldOutput && gamepad1.a) {
                spindexerSlots[outtakePosition] = 0;
                motifIndex = (motifIndex + 1) % motifPattern.length;
                sleep(300);

            } else if (!shouldOutput && gamepad1.b) {
                rotateSpindexerInput();
                sleep(200);
            }

            if (gamepad1.dpad_right) {
                rotateSpindexerInput();
                sleep(200);
            }

            // Manual motif override
            if (gamepad1.dpad_up) {
                currentMotifPattern = (currentMotifPattern + 1) % 3;
                motifPattern = motifPatterns[currentMotifPattern];
                motifIndex = 0;
                motifDetected = true;
                sleep(300);
            }
            if (gamepad1.dpad_down) {
                currentMotifPattern = (currentMotifPattern - 1 + 3) % 3;
                motifPattern = motifPatterns[currentMotifPattern];
                motifIndex = 0;
                motifDetected = true;
                sleep(300);
            }

            if (gamepad1.x) {
                detectMotifFromCamera();
                sleep(300);
            }

            telemetry.addData("Motif", getMotifName(currentMotifPattern));
            telemetry.addData("Progress", motifIndex + "/" + motifPattern.length);
            telemetry.addData("Slots", java.util.Arrays.toString(spindexerSlots));
            telemetry.addData("Required", getBallName(requiredBall));
            telemetry.addData("Ready", shouldOutput);
            telemetry.update();

            sleep(50);
        }
    }

    //intake slots
    double[] intakeslots = {0/360,120/360,240/360};

    //outtake slots
    double[] outtakeslots = {60/360,180/360,300/360};


    //outtake slots
    private void rotateSpindexerInput() {
        int reqIntake = 0; //placeholder
        //spin to required ball
        spindexerServo.setPosition(intakeslots[reqIntake]);
    }
    private void rotateSpindexerOutput(){
        int reqOuttake = 0; //placeholder
        //spin to required ball
        spindexerServo.setPosition(outtakeslots[reqOuttake]);

    }

    /**
     * TODO: Pull motif pattern from AprilTag detector
     * Should set currentMotifPattern (0-2), motifPattern array, and motifDetected flag
     */
    private void detectMotifFromCamera() {
        // Implement your AprilTag integration here
    }

    /**
     * TODO: Pull ball color from color sensor detector
     * @param intakeSystem - fallback source for ball detection
     * @return 0=empty, 1=green, 2=purple
     */
    private int detectIncomingBall(intake intakeSystem) {
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
    private String getBallName(int ballType) {
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
    private String getMotifName(int patternIndex) {
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
    private String getPatternString(int[] pattern) {
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
    private void autoRotateToMatchMotif() {
        int requiredBall = motifPattern[motifIndex];
        int rotations = 0;

        while (rotations < 3 && opModeIsActive()) {
            if (spindexerSlots[outtakePosition] == requiredBall) {
                break;
            }
            rotateSpindexerInput();
            rotations++;
        }
    }
}