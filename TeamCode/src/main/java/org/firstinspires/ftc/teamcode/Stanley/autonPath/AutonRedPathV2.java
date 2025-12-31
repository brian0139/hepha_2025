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
        // Initialize your subsystems here
        // outtake = new outtakeV2(...);
        // intakeSystem = new intake(...);
        // spindexer = new spindexerColor(...);
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

            telemetry.addData("Turret: Has Target", hasTarget);

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
        public boolean run(TelemetryPacket telemetryPacket) {
            if (isComplete) return false;

            boolean hasTarget = outtake.autoturn();

            if (!hasTarget) {
                telemetry.addData("Turret: Status", "No Target");
                return true; // Keep trying
            }

            // Check if aligned
            telemetry.addData("Turret: Status", "Aligning");
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
                telemetry.addData("Hood: Target Angle", targetAngle);
            }

            // Update hood position
            boolean atPosition = outtake.setHood(targetAngle);

            telemetry.addData("Hood: Current Angle", outtake.hoodAngle);
            telemetry.addData("Hood: At Position", atPosition);

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
                telemetry.addData("Flywheel: Target Speed", targetSpeed);
            }

            // Spin and check if at speed
            boolean atSpeed = outtake.spin_flywheel(targetSpeed, tolerance);

            telemetry.addData("Flywheel: Current Speed", outtake.flywheelDriveR.getVelocity());
            telemetry.addData("Flywheel: At Speed", atSpeed);

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
            telemetry.addData("Flywheel: Status", "Stopped");
            return false; // Complete immediately
        }
    }

    // ==================== INTAKE ACTION ====================
    /**
     * Runs intake until pixel detected or timeout
     */
    public class IntakePixel implements Action {
        private final long timeoutMs;

        public IntakePixel(long timeoutMilliseconds) {
            this.timeoutMs = timeoutMilliseconds;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            boolean pixelDetected = intakeSystem.intakeUntilPixel();
            telemetry.addData("Intake: Pixel Detected", pixelDetected);
            telemetry.addData("Intake: Status", pixelDetected ? "Complete" : "Running");

            return !pixelDetected; // Return false when complete
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
            telemetry.addData("Intake: Pixel Detected", detected);
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
            telemetry.addData("Intake: Power", power);
            return false; // Complete immediately
        }
    }

    // ==================== SPINDEXER ACTIONS ====================
    /**
     * Spins spindexer to match current motif pattern
     */
    public class SpinToMotif implements Action {
        private final int motifIndex;
        private boolean initialized = false;

        public SpinToMotif(int motifIndex) {
            this.motifIndex = motifIndex;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                spindexer.initSpin();
                initialized = true;
            }

            boolean complete = spindexer.spinToMotif(motifIndex);

            telemetry.addData("Spindexer: Motif Index", motifIndex);
            telemetry.addData("Spindexer: Status", complete ? "Complete" : "Spinning");

            return !complete; // Return false when complete
        }
    }

    /**
     * Spins spindexer to next empty slot for intake
     */
    public class SpinToIntake implements Action {
        private final int motifIndex;
        private boolean initialized = false;

        public SpinToIntake(int motifIndex) {
            this.motifIndex = motifIndex;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                spindexer.initSpin();
                initialized = true;
            }

            boolean complete = spindexer.spinToIntake(motifIndex);

            telemetry.addData("Spindexer: Motif Index", motifIndex);
            telemetry.addData("Spindexer: Status", complete ? "Complete" : "Spinning");

            return !complete; // Return false when complete
        }
    }

    // ==================== COMPOSITE ACTIONS ====================
    /**
     * Complete shooting sequence: aim turret, adjust hood, spin flywheel, then transfer
     */
    public class ShootSequence implements Action {
        private static final int PHASE_AIM = 0;
        private static final int PHASE_HOOD = 1;
        private static final int PHASE_FLYWHEEL = 2;
        private static final int PHASE_TRANSFER = 3;
        private static final int PHASE_COMPLETE = 4;

        private int currentPhase = PHASE_AIM;

        private final double hoodAngle;
        private final double flywheelSpeed;
        private final int flywheelTolerance;

        public ShootSequence(double hoodAngleDegrees, double flywheelSpeedTPS, int tolerance) {
            this.hoodAngle = hoodAngleDegrees;
            this.flywheelSpeed = flywheelSpeedTPS;
            this.flywheelTolerance = tolerance;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            String phaseName = "";
            switch (currentPhase) {
                case PHASE_AIM: phaseName = "AIM"; break;
                case PHASE_HOOD: phaseName = "HOOD"; break;
                case PHASE_FLYWHEEL: phaseName = "FLYWHEEL"; break;
                case PHASE_TRANSFER: phaseName = "TRANSFER"; break;
                case PHASE_COMPLETE: phaseName = "COMPLETE"; break;
            }
            telemetry.addData("Shoot Sequence: Phase", phaseName);

            switch (currentPhase) {
                case PHASE_AIM:
                    boolean hasTarget = outtake.autoturn();
                    if (hasTarget) {
                        // TODO: Check if actually aligned, not just has target
                        currentPhase = PHASE_HOOD;
                    }
                    return true;

                case PHASE_HOOD:
                    boolean hoodReady = outtake.setHood(hoodAngle);
                    if (hoodReady) {
                        currentPhase = PHASE_FLYWHEEL;
                    }
                    return true;

                case PHASE_FLYWHEEL:
                    boolean flywheelReady = outtake.spin_flywheel(flywheelSpeed, flywheelTolerance);
                    if (flywheelReady) {
                        currentPhase = PHASE_TRANSFER;
                        outtake.transferUp();
                    }
                    return true;

                case PHASE_TRANSFER:
                    outtake.transferDown();
                    currentPhase = PHASE_COMPLETE;
                    return true;

                case PHASE_COMPLETE:
                    return false;
            }

            return false;
        }
    }
}