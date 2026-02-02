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
public class drivetrainMainCRTest extends OpMode {

    // ==================== STATE MACHINE ENUMS ====================
    enum FlywheelState {
        IDLE,
        SPINNING,
        ADJUSTING_SPEED
    }

    enum SpindexerState {
        MANUAL_CONTROL,
        AUTO_POSITION_0,
        AUTO_POSITION_1,
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
    private SpindexerState spindexerState = SpindexerState.AUTO_POSITION_1;
    private TransferState transferState = TransferState.STOPPED;
    private IntakeState intakeState = IntakeState.STOPPED;

    // ==================== CONFIGURATION ====================
    private static final double FLYWHEEL_SENSITIVITY = 10;
    private static final double SPINDEXER_DILATION = 0.01;
    private static final double SPINDEXER_MIN = 0.0;
    private static final double SPINDEXER_MAX = 0.75;
    private static final double SPINDEXER_EPSILON = 0.05;

    private static final double[] SPINDEXER_POSITIONS = {0.0, 0.75};

    private static final double DRIVE_SPEED = 0.7;
    private static final double STRAFE_SPEED_ANALOG = 0.5;
    private static final double STRAFE_SPEED_DPAD = 0.7;
    private static final double TWIST_SPEED = 0.5;
    private static final double SECONDARY_DILATION = 0.25;

    // ==================== WORKING VARIABLES ====================
    private int flywheelSpeed = 2000;
    private int targetSpeed = 0;
    private double spindexerPos = 0.75;
    private int spindexerAutoPos = 1;

    // Button state storage
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

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
        updateHoodControl();
        updateDrivetrain();

        // Update telemetry
        updateTelemetry();

        // Store previous gamepad states
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }

    // ==================== FLYWHEEL STATE MACHINE ====================
    private void updateFlywheelStateMachine() {
        // Adjust speed with right stick
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            int speedChange = (int)(gamepad2.right_stick_y * FLYWHEEL_SENSITIVITY);
            if (flywheelSpeed - speedChange >= 0) {
                flywheelSpeed -= speedChange;
                if (flywheelState == FlywheelState.SPINNING) {
                    flywheelState = FlywheelState.ADJUSTING_SPEED;
                }
            } else {
                flywheelSpeed = 0;
            }
        }

        // Toggle flywheel on/off with Y button
        if (gamepad2.y && !previousGamepad2.y) {
            switch (flywheelState) {
                case IDLE:
                    flywheelState = FlywheelState.SPINNING;
                    targetSpeed = flywheelSpeed;
                    flywheel.setVelocity(targetSpeed);
                    break;

                case SPINNING:
                case ADJUSTING_SPEED:
                    flywheelState = FlywheelState.IDLE;
                    targetSpeed = 0;
                    flywheel.setVelocity(0);
                    break;
            }
        }

        // If adjusting speed while spinning, update velocity
        if (flywheelState == FlywheelState.ADJUSTING_SPEED) {
            targetSpeed = flywheelSpeed;
            flywheel.setVelocity(targetSpeed);
            flywheelState = FlywheelState.SPINNING;
        }
    }

    // ==================== SPINDEXER STATE MACHINE ====================
    private void updateSpindexerStateMachine() {
        boolean rightBumperPressed = (gamepad2.right_bumper && !previousGamepad2.right_bumper) ||
                (gamepad1.right_bumper && !previousGamepad1.right_bumper);
        boolean manualControl = Math.abs(gamepad2.left_stick_x) > 0.1;

        // Check for mode switch to auto positions
        if (rightBumperPressed) {
            spindexerState = SpindexerState.TRANSITIONING;
            spindexerAutoPos++;
            spindexerPos = SPINDEXER_POSITIONS[spindexerAutoPos % 2];
            spindexer.setPosition(spindexerPos);

            // Transition to appropriate auto state
            if (spindexerAutoPos % 2 == 0) {
                spindexerState = SpindexerState.AUTO_POSITION_0;
            } else {
                spindexerState = SpindexerState.AUTO_POSITION_1;
            }
            return;
        }

        // Handle current state
        switch (spindexerState) {
            case MANUAL_CONTROL:
                if (manualControl) {
                    // Continue manual control
                    double adjustment = -gamepad2.left_stick_x * SPINDEXER_DILATION;
                    double newPos = spindexerPos + adjustment;

                    // Clamp position
                    if (newPos >= SPINDEXER_MIN && newPos <= SPINDEXER_MAX) {
                        spindexerPos = newPos;
                    } else if (newPos < SPINDEXER_MIN) {
                        spindexerPos = SPINDEXER_MIN;
                    } else {
                        spindexerPos = SPINDEXER_MAX;
                    }

                    spindexer.setPosition(spindexerPos);
                }
                break;

            case AUTO_POSITION_0:
            case AUTO_POSITION_1:
                if (manualControl) {
                    // Switch to manual control
                    spindexerState = SpindexerState.MANUAL_CONTROL;

                    // Apply manual adjustment
                    double adjustment = -gamepad2.left_stick_x * SPINDEXER_DILATION;
                    double newPos = spindexerPos + adjustment;

                    if (newPos >= SPINDEXER_MIN && newPos <= SPINDEXER_MAX) {
                        spindexerPos = newPos;
                    } else if (newPos < SPINDEXER_MIN) {
                        spindexerPos = SPINDEXER_MIN;
                    } else {
                        spindexerPos = SPINDEXER_MAX;
                    }

                    spindexer.setPosition(spindexerPos);
                }
                break;

            case TRANSITIONING:
                // Check if servo has reached target position
                double currentPos = spindexer.getPosition();
                double targetPos = SPINDEXER_POSITIONS[spindexerAutoPos % 2];
                if (Math.abs(currentPos - targetPos) < SPINDEXER_EPSILON) {
                    if (spindexerAutoPos % 2 == 0) {
                        spindexerState = SpindexerState.AUTO_POSITION_0;
                    } else {
                        spindexerState = SpindexerState.AUTO_POSITION_1;
                    }
                }
                break;
        }
    }

    // ==================== TRANSFER STATE MACHINE ====================
    private void updateTransferStateMachine() {
        boolean xPressed = gamepad2.x && !previousGamepad2.x;

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
            intakeState = IntakeState.INTAKING;
            intake.setPower(intakePower);
        } else if (intakePower < -0.1) {
            intakeState = IntakeState.OUTTAKING;
            intake.setPower(intakePower);
        } else {
            intakeState = IntakeState.STOPPED;
            intake.setPower(0);
        }
    }

    // ==================== HOOD CONTROL ====================
    private void updateHoodControl() {
        hoodServo.setPower(-gamepad2.left_stick_y);
    }

    // ==================== DRIVETRAIN CONTROL ====================
    private void updateDrivetrain() {
        // Base control from analog sticks
        double drive = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = -gamepad1.left_stick_x * STRAFE_SPEED_ANALOG;
        double twist = -gamepad1.right_stick_x * TWIST_SPEED;

        // D-pad overrides for precise movement
        if (gamepad1.dpad_up) drive = DRIVE_SPEED;
        if (gamepad1.dpad_down) drive = -DRIVE_SPEED;
        if (gamepad1.dpad_left) strafe = STRAFE_SPEED_DPAD;
        if (gamepad1.dpad_right) strafe = -STRAFE_SPEED_DPAD;

        // Face buttons for slow precise movement
        if (gamepad1.y) drive = DRIVE_SPEED * SECONDARY_DILATION;
        if (gamepad1.a) drive = -DRIVE_SPEED * SECONDARY_DILATION;
        if (gamepad1.x) strafe = STRAFE_SPEED_DPAD * SECONDARY_DILATION;
        if (gamepad1.b) strafe = -STRAFE_SPEED_DPAD * SECONDARY_DILATION;

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
        if (spindexerState == SpindexerState.MANUAL_CONTROL) {
            telemetry.addLine(String.format("Manual Position: %.3f", spindexerPos));
        } else {
            telemetry.addLine(String.format("Auto Position: %d (%.2f)",
                    spindexerAutoPos % 2, SPINDEXER_POSITIONS[spindexerAutoPos % 2]));
        }
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

        // Drivetrain telemetry
        telemetry.addLine("\n=== DRIVETRAIN ===");
        telemetry.addData("Drive", -gamepad1.left_stick_y * DRIVE_SPEED);
        telemetry.addData("Strafe", -gamepad1.left_stick_x * STRAFE_SPEED_ANALOG);
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