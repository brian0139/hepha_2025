package org.firstinspires.ftc.teamcode.Stanley.autonPath;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alvin.intake;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;

@Autonomous
public class AutonRedPathV2 extends LinearOpMode {

    // Subsystem instances - initialize these in runOpMode
    private outtakeV2 outtake;
    private intake intakeSystem;
    private spindexerColor spindexer;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    // ==================== TURRET ACTION ====================
    /**
     * Auto-aims turret to AprilTag based on team color
     */
    public class TurretAutoAim implements Action {
        private boolean isComplete = false;

        @Override
        public boolean run(TelemetryPacket packet) {
            if (isComplete) return false;

            // autoturn returns false if no valid target or complete
            boolean hasTarget = outtake.autoturn();

            packet.put("Turret: Has Target", hasTarget);

            // Keep running until aligned
            return true;
        }
    }

    /**
     * Auto-aims turret until within threshold, then completes
     */
    public class TurretAutoAimUntilAligned implements Action {
        private final double angleThreshold;
        private boolean isComplete = false;

        public TurretAutoAimUntilAligned(double angleThresholdDegrees) {
            this.angleThreshold = angleThresholdDegrees;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (isComplete) return false;

            boolean hasTarget = outtake.autoturn();

            if (!hasTarget) {
                packet.put("Turret: Status", "No Target");
                return true; // Keep trying
            }

            // Check if aligned
            packet.put("Turret: Status", "Aligning");
            return true;
        }
    }

    // ==================== HOOD ACTION ====================
    /**
     * Sets hood to specific angle and waits until reached
     */
    public class SetHoodAngle implements Action {
        private final double targetAngle;
        private boolean started = false;

        public SetHoodAngle(double angleDegrees) {
            this.targetAngle = angleDegrees;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
                packet.put("Hood: Target Angle", targetAngle);
            }

            // Update hood position
            boolean atPosition = outtake.setHood(targetAngle);

            packet.put("Hood: Current Angle", outtake.hoodAngle);
            packet.put("Hood: At Position", atPosition);

            // Return false when at position (action complete)
            return !atPosition;
        }
    }

    // ==================== FLYWHEEL ACTION ====================
    /**
     * Spins flywheel to target speed and waits until stable
     */
    public class SpinFlywheel implements Action {
        private final double targetSpeed;
        private final int tolerance;
        private boolean started = false;

        public SpinFlywheel(double targetSpeedTicksPerSec, int toleranceTicksPerSec) {
            this.targetSpeed = targetSpeedTicksPerSec;
            this.tolerance = toleranceTicksPerSec;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
                packet.put("Flywheel: Target Speed", targetSpeed);
            }

            // Spin and check if at speed
            boolean atSpeed = outtake.spin_flywheel(targetSpeed, tolerance);

            packet.put("Flywheel: Current Speed", outtake.flywheelDriveR.getVelocity());
            packet.put("Flywheel: At Speed", atSpeed);

            // Return false when at speed (action complete)
            return !atSpeed;
        }
    }

    /**
     * Stops the flywheel
     */
    public class StopFlywheel implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.spin_flywheel(0, 10);
            packet.put("Flywheel: Status", "Stopped");
            return false; // Complete immediately
        }
    }

    // ==================== INTAKE ACTION ====================
    /**
     * Runs intake until pixel detected or timeout, then spins spindexer
     */
    public class IntakePixel implements Action {
        private final long timeoutMs;
        private boolean intakeComplete = false;
        private boolean spindexerComplete = false;

        public IntakePixel(long timeoutMilliseconds) {
            this.timeoutMs = timeoutMilliseconds;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            // Phase 1: Run intake until pixel detected
            if (!intakeComplete) {
                boolean pixelDetected = intakeSystem.intakeUntilPixel();
                packet.put("Intake: Pixel Detected", pixelDetected);

                if (pixelDetected) {
                    intakeComplete = true;
                    packet.put("Intake: Status", "Complete - Starting Spindexer");
                } else {
                    packet.put("Intake: Status", "Running");
                    return true; // Still intaking
                }
            }

            // Phase 2: Spin spindexer on detection
            if (!spindexerComplete) {
                boolean spinComplete = spindexer.spinOnDetection();
                packet.put("Spindexer: Status", spinComplete ? "Complete" : "Spinning");

                if (spinComplete) {
                    spindexerComplete = true;
                    return false; // Both phases complete
                }
                return true; // Still spinning spindexer
            }

            return false; // Should never reach here
        }
    }

    /**
     * Just runs intake without spindexer
     */
    public class RunIntake implements Action {
        private final long timeoutMs;

        public RunIntake(long timeoutMilliseconds) {
            this.timeoutMs = timeoutMilliseconds;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            boolean detected = intakeSystem.intakeUntilPixel();
            packet.put("Intake: Pixel Detected", detected);
            return !detected; // Return false when complete
        }
    }

    /**
     * Manually control intake power
     */
    public class SetIntakePower implements Action {
        private final double power;

        public SetIntakePower(double power) {
            this.power = power;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            intakeSystem.setPower(power);
            packet.put("Intake: Power", power);
            return false; // Complete immediately
        }
    }

    // ==================== SPINDEXER ACTIONS ====================
    /**
     * Spins spindexer to match current motif pattern
     */
    public class SpinToMotif implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            boolean complete = spindexer.spinToMotif();

            packet.put("Spindexer: Motif Index", spindexer.motifIndex);
            packet.put("Spindexer: Timeout Count", spindexer.timeout);
            packet.put("Spindexer: Status", complete ? "Complete" : "Spinning");

            return !complete; // Return false when complete
        }
    }

    /**
     * Spins spindexer to next empty slot for intake
     */
    public class SpinToIntake implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            boolean complete = spindexer.spinToIntake();

            packet.put("Spindexer: Slots", java.util.Arrays.toString(spindexer.spindexerSlots));
            packet.put("Spindexer: Timeout", spindexer.timeout);
            packet.put("Spindexer: Status", complete ? "Complete" : "Spinning");

            return !complete; // Return false when complete
        }
    }

    /**
     * Spins on color detection (automated response to intake)
     */
    public class SpinOnDetection implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            boolean complete = spindexer.spinOnDetection();

            packet.put("Spindexer: Status", complete ? "Complete" : "Monitoring");

            return !complete; // Return false when complete
        }
    }

    // ==================== COMPOSITE ACTIONS ====================
    /**
     * Complete shooting sequence: aim turret, adjust hood, spin flywheel, then transfer
     */
    public static class ShootSequence implements Action {
        private enum Phase { AIM, HOOD, FLYWHEEL, TRANSFER, COMPLETE }
        private Phase currentPhase = Phase.AIM;

        private outtakeV2 outtake;

        private final double hoodAngle;
        private final double flywheelSpeed;
        private final int flywheelTolerance;

        public ShootSequence(outtakeV2 outtake, double hoodAngleDegrees, double flywheelSpeedTPS, int tolerance) {
            this.outtake = outtake;
            this.hoodAngle = hoodAngleDegrees;
            this.flywheelSpeed = flywheelSpeedTPS;
            this.flywheelTolerance = tolerance;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            packet.put("Shoot Sequence: Phase", currentPhase.toString());

            switch (currentPhase) {
                case AIM:
                    boolean hasTarget = outtake.autoturn();
                    if (hasTarget) {
                        // TODO: Check if actually aligned, not just has target
                        currentPhase = Phase.HOOD;
                    }
                    return true;

                case HOOD:
                    boolean hoodReady = outtake.setHood(hoodAngle);
                    if (hoodReady) {
                        currentPhase = Phase.FLYWHEEL;
                    }
                    return true;

                case FLYWHEEL:
                    boolean flywheelReady = outtake.spin_flywheel(flywheelSpeed, flywheelTolerance);
                    if (flywheelReady) {
                        currentPhase = Phase.TRANSFER;
                        outtake.transferUp();
                    }
                    return true;

                case TRANSFER:
                    // Wait a bit for transfer to complete
                    outtake.transferDown();
                    currentPhase = Phase.COMPLETE;
                    return true;

                case COMPLETE:
                    return false;
            }

            return false;
        }
    }
}