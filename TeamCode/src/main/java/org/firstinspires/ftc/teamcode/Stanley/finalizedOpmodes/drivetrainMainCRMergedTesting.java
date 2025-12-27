package org.firstinspires.ftc.teamcode.Stanley.finalizedOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;

@TeleOp
public class drivetrainMainCRMergedTesting extends OpMode {

    // ==================== STATE MACHINE ENUMS ====================
    enum FlywheelState {
        IDLE,
        SPINNING,
        SPEED_INCREASING,
        SPEED_DECREASING
    }

    enum SpindexerState {
        POSITION_0,
        POSITION_1,
        TRANSITIONING
    }

    enum TransferState {
        STOPPED,
        RUNNING
    }

    enum IntakeState {
        STOPPED,
        INTAKING,
        OUTTAKING
    }

    enum HoodState {
        STOPPED,
        MOVING_UP,
        MOVING_DOWN
    }

    // ==================== HARDWARE DECLARATION ====================
    // Drivetrain motors
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    // Other motors
    private DcMotorEx flywheel = null;
    private DcMotorEx intake = null;
    private DcMotor transfer = null;

    // Servos
    private CRServo hoodServo = null;
    private Servo spindexer = null;

    // ==================== STATE VARIABLES ====================
    private FlywheelState flywheelState = FlywheelState.IDLE;
    private SpindexerState spindexerState = SpindexerState.POSITION_1;
    private TransferState transferState = TransferState.STOPPED;
    private IntakeState intakeState = IntakeState.STOPPED;
    private HoodState hoodState = HoodState.STOPPED;

    // ==================== CONFIGURATION ====================
    private static final double FLYWHEEL_SENSITIVITY = 10;
    private static final double HOOD_SPEED = 0.5;
    private static final double DRIVE_SPEED = 0.7;
    private static final double STRAFE_SPEED = 0.5;
    private static final double TWIST_SPEED = 0.5;

    private static final double[] SPINDEXER_POSITIONS = {0.0, 0.75};
    private static final double SPINDEXER_EPSILON = 0.01;

    // ==================== WORKING VARIABLES ====================
    private int flywheelSpeed = 2000;
    private int targetSpeed = 0;
    private double spindexerPos = 0.75;
    private int spindexerAutoPos = 1;

    // Button state storage
    private Gamepad previousGamepad1 = new Gamepad();

    // ==================== INITIALIZATION ====================
    @Override
    public void init() {
        // Initialize drivetrain motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Reverse motor directions where needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize other motors
        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = (DcMotorEx) hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "par1");

        // Initialize servos
        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");
        spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        spindexer.setPosition(spindexerPos);

        telemetry.addLine("Robot Initialized and Ready");
        telemetry.update();
    }

    // ==================== MAIN LOOP ====================
    @Override
    public void loop() {
        // Update all state machines
        updateFlywheelStateMachine();
        updateSpindexerStateMachine();
        updateTransferStateMachine();
        updateIntakeStateMachine();
        updateHoodStateMachine();
        updateDrivetrain();

        // Update telemetry
        updateTelemetry();

        // Store previous gamepad state
        previousGamepad1.copy(gamepad1);
    }

    // ==================== FLYWHEEL STATE MACHINE ====================
    private void updateFlywheelStateMachine() {
        // Speed adjustment with D-pad
        boolean speedChanged = false;

        if (gamepad1.dpad_up) {
            flywheelSpeed += FLYWHEEL_SENSITIVITY;
            speedChanged = true;
            if (flywheelState == FlywheelState.SPINNING) {
                flywheelState = FlywheelState.SPEED_INCREASING;
            }
        } else if (gamepad1.dpad_down) {
            if (flywheelSpeed - FLYWHEEL_SENSITIVITY >= 0) {
                flywheelSpeed -= FLYWHEEL_SENSITIVITY;
            } else {
                flywheelSpeed = 0;
            }
            speedChanged = true;
            if (flywheelState == FlywheelState.SPINNING) {
                flywheelState = FlywheelState.SPEED_DECREASING;
            }
        }

        // Toggle flywheel on/off with Y button
        if (gamepad1.y && !previousGamepad1.y) {
            switch (flywheelState) {
                case IDLE:
                case SPEED_INCREASING:
                case SPEED_DECREASING:
                    if (flywheelState == FlywheelState.IDLE) {
                        flywheelState = FlywheelState.SPINNING;
                        targetSpeed = flywheelSpeed;
                        flywheel.setVelocity(targetSpeed);
                    } else {
                        flywheelState = FlywheelState.IDLE;
                        targetSpeed = 0;
                        flywheel.setVelocity(0);
                    }
                    break;

                case SPINNING:
                    flywheelState = FlywheelState.IDLE;
                    targetSpeed = 0;
                    flywheel.setVelocity(0);
                    break;
            }
        }

        // Update velocity if speed changed while spinning
        if (speedChanged && (flywheelState == FlywheelState.SPEED_INCREASING ||
                flywheelState == FlywheelState.SPEED_DECREASING)) {
            targetSpeed = flywheelSpeed;
            flywheel.setVelocity(targetSpeed);
            flywheelState = FlywheelState.SPINNING;
        }
    }

    // ==================== SPINDEXER STATE MACHINE ====================
    private void updateSpindexerStateMachine() {
        boolean rightBumperPressed = gamepad1.right_bumper && !previousGamepad1.right_bumper;

        if (rightBumperPressed) {
            switch (spindexerState) {
                case POSITION_0:
                    spindexerState = SpindexerState.TRANSITIONING;
                    spindexerAutoPos++;
                    spindexerPos = SPINDEXER_POSITIONS[spindexerAutoPos % 2];
                    spindexer.setPosition(spindexerPos);
                    spindexerState = SpindexerState.POSITION_1;
                    break;

                case POSITION_1:
                    spindexerState = SpindexerState.TRANSITIONING;
                    spindexerAutoPos++;
                    spindexerPos = SPINDEXER_POSITIONS[spindexerAutoPos % 2];
                    spindexer.setPosition(spindexerPos);
                    spindexerState = SpindexerState.POSITION_0;
                    break;

                case TRANSITIONING:
                    // Wait for transition to complete
                    break;
            }
        }

        // Check if servo has reached target position (optional refinement)
        double currentPos = spindexer.getPosition();
        if (spindexerState == SpindexerState.TRANSITIONING) {
            double targetPos = SPINDEXER_POSITIONS[spindexerAutoPos % 2];
            if (Math.abs(currentPos - targetPos) < SPINDEXER_EPSILON) {
                spindexerState = (spindexerAutoPos % 2 == 0) ?
                        SpindexerState.POSITION_0 : SpindexerState.POSITION_1;
            }
        }
    }

    // ==================== TRANSFER STATE MACHINE ====================
    private void updateTransferStateMachine() {
        boolean xPressed = gamepad1.x && !previousGamepad1.x;

        if (xPressed) {
            switch (transferState) {
                case STOPPED:
                    transferState = TransferState.RUNNING;
                    transfer.setPower(1.0);
                    break;

                case RUNNING:
                    transferState = TransferState.STOPPED;
                    transfer.setPower(0.0);
                    break;
            }
        }
    }

    // ==================== INTAKE STATE MACHINE ====================
    private void updateIntakeStateMachine() {
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        if (intakePower > 0.1) {
            if (intakeState != IntakeState.INTAKING) {
                intakeState = IntakeState.INTAKING;
            }
            intake.setPower(intakePower);
        } else if (intakePower < -0.1) {
            if (intakeState != IntakeState.OUTTAKING) {
                intakeState = IntakeState.OUTTAKING;
            }
            intake.setPower(intakePower);
        } else {
            if (intakeState != IntakeState.STOPPED) {
                intakeState = IntakeState.STOPPED;
            }
            intake.setPower(0);
        }
    }

    // ==================== HOOD STATE MACHINE ====================
    private void updateHoodStateMachine() {
        if (gamepad1.dpad_right) {
            hoodState = HoodState.MOVING_UP;
            hoodServo.setPower(HOOD_SPEED);
        } else if (gamepad1.dpad_left) {
            hoodState = HoodState.MOVING_DOWN;
            hoodServo.setPower(-HOOD_SPEED);
        } else {
            hoodState = HoodState.STOPPED;
            hoodServo.setPower(0);
        }
    }

    // ==================== DRIVETRAIN CONTROL ====================
    private void updateDrivetrain() {
        // Base control from analog sticks
        double drive = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = -gamepad1.left_stick_x * STRAFE_SPEED;
        double twist = -gamepad1.right_stick_x * TWIST_SPEED;

        // Calculate mecanum wheel speeds
        double[] speeds = {
                (drive - strafe - twist), // leftFront
                (drive + strafe + twist), // rightFront
                (drive + strafe - twist), // leftBack
                (drive - strafe + twist)  // rightBack
        };

        // Normalize speeds if any exceed 1.0
        double max = Math.abs(speeds[0]);
        for (int i = 1; i < speeds.length; i++) {
            if (Math.abs(speeds[i]) > max) {
                max = Math.abs(speeds[i]);
            }
        }

        if (max > 1.0) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] /= max;
            }
        }

        // Apply speeds to motors
        leftFront.setPower(speeds[0]);
        rightFront.setPower(speeds[1]);
        leftBack.setPower(speeds[2]);
        rightBack.setPower(speeds[3]);
    }

    // ==================== TELEMETRY ====================
    private void updateTelemetry() {
        // Flywheel telemetry
        telemetry.addLine("=== FLYWHEEL ===");
        telemetry.addLine("State: " + flywheelState);
        telemetry.addLine(String.format("Set Speed: %d ticks/s (%.0f RPM)",
                flywheelSpeed, flywheelSpeed * 60.0 / 28.0));
        telemetry.addLine(String.format("Target: %d ticks/s (%.0f RPM)",
                targetSpeed, targetSpeed * 60.0 / 28.0));
        telemetry.addLine(String.format("Actual: %.0f ticks/s (%.0f RPM)",
                flywheel.getVelocity(), flywheel.getVelocity() * 60.0 / 28.0));

        // Spindexer telemetry
        telemetry.addLine("\n=== SPINDEXER ===");
        telemetry.addLine("State: " + spindexerState);
        telemetry.addLine(String.format("Position: %d (%.2f)",
                spindexerAutoPos % 2, SPINDEXER_POSITIONS[spindexerAutoPos % 2]));
        telemetry.addData("Real Position", spindexer.getPosition());

        // Transfer telemetry
        telemetry.addLine("\n=== TRANSFER ===");
        telemetry.addLine("State: " + transferState);
        if (transferState == TransferState.RUNNING) {
            telemetry.addLine("Position: Up (Running)");
        } else {
            telemetry.addLine("Position: Stopped");
        }

        // Intake telemetry
        telemetry.addLine("\n=== INTAKE ===");
        telemetry.addLine("State: " + intakeState);

        // Hood telemetry
        telemetry.addLine("\n=== HOOD ===");
        telemetry.addLine("State: " + hoodState);

        // Drivetrain telemetry
        telemetry.addLine("\n=== DRIVETRAIN ===");
        telemetry.addData("Drive", -gamepad1.left_stick_y * DRIVE_SPEED);
        telemetry.addData("Strafe", -gamepad1.left_stick_x * STRAFE_SPEED);
        telemetry.addData("Twist", -gamepad1.right_stick_x * TWIST_SPEED);

        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean shutdown
        flywheel.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
        hoodServo.setPower(0);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}