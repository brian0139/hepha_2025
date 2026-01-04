package org.firstinspires.ftc.teamcode.Stanley.autonPath;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alvin.intake;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;

@Autonomous
public class AutonRedPathV2 extends LinearOpMode {
    outtakeV2 outtake;
    intake intakeSystem;
    spindexerColor spindexer;
    CRServo spindexerServo=null;
    ElapsedTime timer=new ElapsedTime();
    DcMotor intakeMotor=null;
    DcMotorEx transfer=null;
    DcMotorEx flywheel=null;
    DcMotorEx flywheelR=null;
    CRServo hood=null;
    AnalogInput hoodSensor=null;
    Pose2d beginPose=new Pose2d(-57.5, 43.5, Math.toRadians(360-54));
    MecanumDrive drive=null;
    NormalizedColorSensor intakeSensor;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        outtake = new outtakeV2(hardwareMap,flywheel,flywheelR,null,null,null,null,null,hood,hoodSensor,transfer,true);
        intakeSystem = new intake(hardwareMap,"intake","intakeSensor");
        spindexer=new spindexerColor(spindexerServo,intakeMotor,hardwareMap);
        spindexerServo=hardwareMap.crservo.get("spindexerServo");
        intakeMotor=hardwareMap.dcMotor.get("intake");
        transfer=(DcMotorEx) hardwareMap.dcMotor.get("par1");
        flywheel=(DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        flywheelR=(DcMotorEx) hardwareMap.dcMotor.get("flywheelR");
        hood=hardwareMap.crservo.get("hoodServo");
        hoodSensor=hardwareMap.get(AnalogInput.class,"hoodAnalog");
        drive=new MecanumDrive(hardwareMap,beginPose);
        intakeSensor=hardwareMap.get(NormalizedColorSensor.class,"intakeSensor");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        final Vector2d shootingPos=new Vector2d(-34,23);
        final double shootingAngle=Math.toRadians(120);
        final double intakeFinishy =36;
        final double intakeStarty=13;

        dashboard=FtcDashboard.getInstance();
        dashboardTelemetry=dashboard.getTelemetry();
        dashboardTelemetry.addData("Hood: Current Angle",0);
        dashboardTelemetry.addData("Power",0);
        dashboardTelemetry.addData("Voltage",0);
        dashboardTelemetry.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)

                            .strafeToLinearHeading(shootingPos, shootingAngle+Math.toRadians(7))
                            .stopAndAdd(new ScanMotif())
                            .waitSeconds(1)
                            //Start Flywheel
                            .stopAndAdd(new SpinFlywheel(670,50))
                            //Shooting Sequence
                            .stopAndAdd(new TurretAutoAimUntilAligned())
                            .stopAndAdd(new transferUp())
                            .stopAndAdd(new RunIntake())
                            .waitSeconds(3)
                            .stopAndAdd(new startspindexer())
                            .waitSeconds(3)
                            //Stop Sequence
                            .stopAndAdd(new StopFlywheel())
                            .stopAndAdd(new transferOff())
                            .stopAndAdd(new stopspindexer())
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new StopTurret())
                            .strafeToLinearHeading(new Vector2d(-14, intakeStarty), Math.toRadians(360-270))
                            //Start Intake Code
                            .stopAndAdd(new RunIntake())
                            .stopAndAdd(new startspindexer())

                            .strafeTo(new Vector2d(-14, intakeFinishy))
                            .strafeTo(new Vector2d(-14,intakeFinishy+10))

                            //Stop Intake
                            .waitSeconds(3)
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new stopspindexer())
                            .strafeToLinearHeading(new Vector2d(-37,23), shootingAngle)
                            //Start Flywheel
                            .stopAndAdd(new SpinFlywheel(500,50))
                            //Shoot Sequence
                            .stopAndAdd(new TurretAutoAimUntilAligned())
                            .stopAndAdd(new transferUp())
                            .stopAndAdd(new RunIntake())
                            .waitSeconds(3)
                            .stopAndAdd(new startspindexer())
                            .waitSeconds(3)
                            //Stop Sequence \
                            .stopAndAdd(new StopFlywheel())
                            .stopAndAdd(new transferOff())
                            .stopAndAdd(new stopspindexer())
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new StopTurret())
                            .strafeToLinearHeading(new Vector2d(14, intakeStarty-7), Math.toRadians(360-270))
                            //Start Intake
                            .stopAndAdd(new RunIntake())
                            .stopAndAdd(new startspindexer())
                            .strafeTo(new Vector2d(14, intakeFinishy+3))
                            //Stop Intake
                            .waitSeconds(3)
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new stopspindexer())
                            .strafeToLinearHeading(new Vector2d(-35,23), shootingAngle)
                            //Start Flywheel
                            .stopAndAdd(new SpinFlywheel(500,50))
                            //Shoot Sequence
                            .stopAndAdd(new TurretAutoAimUntilAligned())
                            .stopAndAdd(new transferUp())
                            .stopAndAdd(new RunIntake())
                            .waitSeconds(3)
                            .stopAndAdd(new startspindexer())
                            .waitSeconds(3)
                            //Stop Sequence
                            .stopAndAdd(new StopFlywheel())
                            .stopAndAdd(new transferOff())
                            .stopAndAdd(new stopspindexer())
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new StopTurret())
                            //Start Intake
                            .strafeToLinearHeading(new Vector2d(36, intakeStarty-10), Math.toRadians(360-270))
                            .stopAndAdd(new RunIntake())
                            .stopAndAdd(new startspindexer())
                            .strafeTo(new Vector2d(36, intakeFinishy+3))
                            //Stop Intake
                            .waitSeconds(3)
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new stopspindexer())
                            .strafeToLinearHeading(shootingPos, shootingAngle)
                            //Start Flywheel
                            .stopAndAdd(new SpinFlywheel(500,50))
                            //Shoot Sequence
                            .stopAndAdd(new TurretAutoAimUntilAligned())
                            .stopAndAdd(new transferUp())
                            .stopAndAdd(new RunIntake())
                            .waitSeconds(3)
                            .stopAndAdd(new startspindexer())
                            .waitSeconds(3)
                            //Stop Sequence
                            .stopAndAdd(new StopFlywheel())
                            .stopAndAdd(new transferOff())
                            .stopAndAdd(new stopspindexer())
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new StopTurret())
                            .build());
            break;
        }
        telemetry.addData("Status", "All Tests Complete");
        telemetry.update();
    }

    /**
     * Helper method to run an action with timeout
     */
    private void runAction(Action action, long timeoutMs) {
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(action)
                        .build()
        );
    }

    // ==================== TURRET ACTION ====================
    /**
     * Auto-aims turret to AprilTag based on team color
     */

    //TODO: Integrate motif recognition into auton and make it have the ability to recognize the motif not the tower
    public class ScanMotif implements Action {
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

    //TODO: Integrate tower recognition into auton and make it have the ability to recognize the tower not the motif
    public class TurretAutoAimUntilAligned implements Action {
        private boolean isComplete = false;
        private final double alignmentThreshold = 2.0; // degrees, adjust as needed

        @Override
        public boolean run(TelemetryPacket telemetryPacket) {
            if (isComplete) return false;

            boolean hasTarget = outtake.autoturn();

            if (!hasTarget) {
                telemetry.addData("Turret: Status", "No Target");
                // Optionally complete after some attempts or keep trying
                return true;
            }

            // Check if aligned by examining heading error
            double headingError = Math.abs(outtake.apriltag.getYaw());
            if (headingError < alignmentThreshold) {
                outtake.turretServo.setPower(0); // Stop the turret
                isComplete = true;
                telemetry.addData("Turret: Status", "Aligned!");
                return false; // Action complete
            }

            telemetry.addData("Turret: Status", "Aligning");
            return true;
        }
    }

    public class StopTurret implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.turretServo.setPower(0);
            telemetry.addData("Turret: Status", "Stopped");
            return false; // Complete immediately
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
                outtake.initHoodAngleBlocking();
                outtake.hoodPID.init();
//                outtake.updateHoodAngle();
            }

            // Update hood position
            boolean atPosition = outtake.setHood(targetAngle);

            telemetry.addData("Hood: Current Angle", outtake.hoodAngle* outtake.servoDegPerRot);
            dashboardTelemetry.addData("Hood: Current Angle", outtake.hoodAngle* outtake.servoDegPerRot);
            dashboardTelemetry.addData("Power",outtake.hoodPID.power);
            dashboardTelemetry.addData("Voltage",outtake.hoodSensor.getVoltage());
            dashboardTelemetry.update();
            telemetry.addData("Hood: At Position", atPosition);
            telemetry.update();

            // Return false when at position (action complete)
            return true;
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
            return false;
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

    // ==================== TRANSFER ACTION ====================
    public class transferUp implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.transferUp();
            telemetry.addData("Transfer: Status", "Up");
            return false; // Complete immediately
        }
    }

    public class transferOff implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.transferDown();
            telemetry.addData("Transfer: Status", "Off");
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
            telemetry.update();

            return !pixelDetected; // Return false when complete
        }
    }

    /**
     * Just runs intake without spindexer
     */
    public class RunIntake implements Action {

        @Override
        public boolean run(TelemetryPacket packet) {
            intakeSystem.start();
            return false;
        }
    }

    public class StopIntake implements Action {

        @Override
        public boolean run(TelemetryPacket packet) {
            intakeSystem.stop();
            return false;
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

    public class startspindexer implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            spindexerServo.setPower(1);
            return false;
        }
    }

    public class stopspindexer implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            spindexerServo.setPower(0);
            return false;
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
}