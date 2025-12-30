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
    public int[] spindexerSlots={0,0,0}; //0 none, 1 green, 2 purple
    public int[][] motifPatterns = {
            {1, 2, 2},
            {2, 1, 2},
            {2, 2, 1}
    };
    public int motifIndex=0;
    public int[] dummyMotif={1,2,2};
    private boolean isSpinningToMotif = false;
    private boolean detectedLastLoopMotif = false;
    private int timeoutMotif = 0;
    private boolean isBrakingMotif = false;
    private ElapsedTime brakeTimerMotif = new ElapsedTime();
    public int[] currentMotifPattern=null;
    private boolean isSpinning = false;
    private boolean seenEmpty = false;
    private int detectedColor = 0;
    private boolean isSpinningToIntake = false;
    private boolean detectedLastLoop = true;
    private boolean seenEmptyIntake = false;
    private ElapsedTime emptyTimerIntake = new ElapsedTime();
    private ElapsedTime spinStartTime = new ElapsedTime();
    private boolean isBraking = false;
    private ElapsedTime brakeTimer = new ElapsedTime();
    public spindexerColor(CRServo spindexerServo, HardwareMap hardwareMap){
        this.spindexerServo=spindexerServo;
        outtakesensor=new colorSensor(hardwareMap,"outtakeSensor");
        intakesensor=new colorSensor(hardwareMap, "intakeSensor");
    }

    public boolean spinToMotif() {
        int nextmotif = dummyMotif[motifIndex];

        // If not currently spinning, start the process
        if (!isSpinningToMotif) {
            timeoutMotif = 0;
            detectedLastLoopMotif = false;
            isBrakingMotif = false;
            isSpinningToMotif = true;
            spindexerServo.setPower(0.75);
            return false;  // Not done yet
        }

        // Check if we're currently braking
        if (isBrakingMotif) {
            if (brakeTimerMotif.milliseconds() >= 400) {
                spindexerServo.setPower(0);
                isSpinningToMotif = false;

                // Check if we succeeded or failed
                if (outtakesensor.getDetected() == nextmotif && timeoutMotif < 3) {
                    motifIndex++;
                    motifIndex %= 3;
                    return true;  // Success!
                }
                return false;  // Failed
            }
            return false;  // Still braking
        }

        // Check if we found the target motif or timed out
        if (outtakesensor.getDetected() == nextmotif || timeoutMotif >= 3) {
            // Start braking
            isBrakingMotif = true;
            brakeTimerMotif.reset();
            spindexerServo.setPower(-0.01);
            timeout = timeoutMotif;  // Update global timeout
            return false;  // Not done yet, braking
        }

        // Continue spinning
        spindexerServo.setPower(0.75);

        // Count slots passed
        if ((outtakesensor.getDetected() == 1 || outtakesensor.getDetected() == 2) && !detectedLastLoopMotif) {
            timeoutMotif++;
            detectedLastLoopMotif = true;
        } else if (outtakesensor.getDetected() == 0) {
            detectedLastLoopMotif = false;
        }

        return false;  // Still spinning
    }
    public boolean spinToIntake(){
        // If not currently spinning, start the process
        if (!isSpinningToIntake) {
            timeout = 0;
            detectedLastLoop = true;
            seenEmptyIntake = false;
            isBraking = false;
            spinStartTime.reset();
            isSpinningToIntake = true;
            spindexerServo.setPower(0.4);
            return false;  // Not done yet
        }

        // Check timeout conditions
        if (timeout >= 3 || spinStartTime.milliseconds() >= 2000) {
            // Timeout - apply brake and finish
            if (!isBraking) {
                isBraking = true;
                brakeTimer.reset();
                spindexerServo.setPower(-0.01);
            } else if (brakeTimer.milliseconds() >= 400) {
                spindexerServo.setPower(0);
                isSpinningToIntake = false;
                return false;  // Failed
            }
            return false;
        }

        int currentReading = intakesensor.getDetected();

        // Count when transitioning from filled (1 or 2) to empty (0)
        if (currentReading == 0 && detectedLastLoop == true) {
            timeout++;
            detectedLastLoop = false;
        } else if (currentReading != 0) {
            detectedLastLoop = true;
        }

        // Handle empty readings
        if (currentReading == 0) {
            if (!seenEmptyIntake) {
                emptyTimerIntake.reset();
                seenEmptyIntake = true;
            }

            // Wait 300ms to confirm it's a real empty slot, not a gap
            if (emptyTimerIntake.milliseconds() >= 300) {
                spindexerServo.setPower(0.1);  // Very slow for final positioning
                if (emptyTimerIntake.milliseconds() >= 400) {
                    // Found empty slot - apply brake
                    if (!isBraking) {
                        isBraking = true;
                        brakeTimer.reset();
                        spindexerServo.setPower(-0.01);
                    } else if (brakeTimer.milliseconds() >= 400) {
                        spindexerServo.setPower(0);

                        // Update slots
                        for (int i = 0; i < 3; i++) {
                            if (spindexerSlots[i] != 0) {
                                spindexerSlots[i] = intakesensor.getDetected();
                            }
                        }

                        isSpinningToIntake = false;
                        return true;  // Success!
                    }
                }
            } else {
                spindexerServo.setPower(0.4);  // Continue at normal speed
            }
        } else {
            spindexerServo.setPower(0.4);  // Normal speed on filled slots
            seenEmptyIntake = false;
        }

        return false;  // Still spinning
    }
    // Add these as class variables at the top of your spindexerColor class


    public boolean spinOnDetection() {
        // If not currently spinning, check if we should start
        if (!isSpinning) {
            int currentReading = intakesensor.getDetected();
            if (currentReading == 1 || currentReading == 2) {
                // Start spinning
                isSpinning = true;
                seenEmpty = false;
                detectedColor = currentReading;
                nonetime.reset();
                spindexerServo.setPower(0.75);
            }
            return false;  // Not done yet or nothing to do
        }

        // Currently spinning - check if we found empty slot
        int currentReading = intakesensor.getDetected();

        if (currentReading == 0) {
            if (!seenEmpty) {
                // Just started seeing empty, start timer
                seenEmpty = true;
                nonetime.reset();
            } else if (nonetime.milliseconds() >= 400) {
                // Been empty for 400ms, apply brake and finish
                timer.reset();
                spindexerServo.setPower(-0.01);
                if (timer.milliseconds() >= 400) {
                    spindexerServo.setPower(0);
                    isSpinning = false;  // Reset state
                    return true;  // Done!
                }
            }
        } else {
            // Seeing a ball, reset empty tracking and keep spinning
            seenEmpty = false;
            spindexerServo.setPower(0.75);
        }

        return false;  // Still spinning
    }
}
