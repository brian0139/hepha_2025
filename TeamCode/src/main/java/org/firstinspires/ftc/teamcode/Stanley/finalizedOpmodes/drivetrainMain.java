package org.firstinspires.ftc.teamcode.Stanley.finalizedOpmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;

@TeleOp
public class drivetrainMain extends OpMode {

    // ==================== STATE MACHINE ENUMS ====================
    enum FlywheelState {
        IDLE,
        SPINNING,
        ADJUSTING_SPEED
    }

    enum SpindexerState {
        INTAKE_MODE,
        OUTTAKE_MODE,
        TRANSITIONING
    }

    enum TransferState {
        DOWN,
        UP,
        MOVING
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

    // Servos
    private CRServo hoodServo = null;
    private Servo spindexer = null;
    private Servo transfer = null;

    // ==================== STATE VARIABLES ====================
    private FlywheelState flywheelState = FlywheelState.IDLE;
    private SpindexerState spindexerState = SpindexerState.INTAKE_MODE;
    private TransferState transferState = TransferState.DOWN;
    private IntakeState intakeState = IntakeState.STOPPED;

    // ==================== CONFIGURATION ====================
    private static final double FLYWHEEL_SENSITIVITY = 10;
    private static final double DRIVE_SPEED = 0.7;
    private static final double STRAFE_SPEED = 0.5;
    private static final double TWIST_SPEED = 0.5;
    private static final double SECONDARY_DILATION = 0.25;

    private static final double[] OUTTAKE_SLOTS = {0.65, 1.0, 0.26};
    private static final double[] INTAKE_SLOTS = {0.05, 0.44, 0.83};
    private static final double[] TRANSFER_POSITIONS = {0.68, 0.875};
    private static final int TRANSFER_DOWN = 1;
    private static final int TRANSFER_UP = 0;

    // ==================== WORKING VARIABLES ====================
    private int flywheelSpeed = 2000;
    private int targetSpeed = 0;
    private int outtakePos = 0;
    private int intakePos = 0;

    // Button state storage
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    private boolean previousFlywheelToggle = false;

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

        // Initialize servos
        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");
        spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        transfer = hardwareMap.get(Servo.class, "transferServo");

        // Set initial positions
        transfer.setPosition(TRANSFER_POSITIONS[TRANSFER_DOWN]);
        transferState = TransferState.DOWN;

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
        boolean leftBumperPressed = (gamepad2.left_bumper && !previousGamepad2.left_bumper) ||
                (gamepad1.left_bumper && !previousGamepad1.left_bumper);

        // Only allow spindexer movement when transfer is down
        if (transferState != TransferState.DOWN) {
            return;
        }

        switch (spindexerState) {
            case INTAKE_MODE:
                if (rightBumperPressed) {
                    spindexerState = SpindexerState.TRANSITIONING;
                    outtakePos++;
                    spindexer.setPosition(OUTTAKE_SLOTS[outtakePos % 3]);
                    spindexerState = SpindexerState.OUTTAKE_MODE;
                } else if (leftBumperPressed) {
                    intakePos++;
                    spindexer.setPosition(INTAKE_SLOTS[intakePos % 3]);
                }
                break;

            case OUTTAKE_MODE:
                if (leftBumperPressed) {
                    spindexerState = SpindexerState.TRANSITIONING;
                    intakePos++;
                    spindexer.setPosition(INTAKE_SLOTS[intakePos % 3]);
                    spindexerState = SpindexerState.INTAKE_MODE;
                } else if (rightBumperPressed) {
                    outtakePos++;
                    spindexer.setPosition(OUTTAKE_SLOTS[outtakePos % 3]);
                }
                break;

            case TRANSITIONING:
                // Transition is instant for servos, so this state is brief
                break;
        }
    }

    // ==================== TRANSFER STATE MACHINE ====================
    private void updateTransferStateMachine() {
        // Only allow transfer control in outtake mode
        if (spindexerState != SpindexerState.OUTTAKE_MODE) {
            return;
        }

        switch (transferState) {
            case DOWN:
                if (gamepad2.x) {
                    transferState = TransferState.MOVING;
                    transfer.setPosition(TRANSFER_POSITIONS[TRANSFER_UP]);
                    transferState = TransferState.UP;
                }
                break;

            case UP:
                if (!gamepad2.x) {
                    transferState = TransferState.MOVING;
                    transfer.setPosition(TRANSFER_POSITIONS[TRANSFER_DOWN]);
                    transferState = TransferState.DOWN;
                }
                break;

            case MOVING:
                // Servo movement is instant, so this state is brief
                break;
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
        double strafe = -gamepad1.left_stick_x * STRAFE_SPEED;
        double twist = -gamepad1.right_stick_x * TWIST_SPEED;

        // D-pad overrides for precise movement
        if (gamepad1.dpad_up) drive = DRIVE_SPEED;
        if (gamepad1.dpad_down) drive = -DRIVE_SPEED;
        if (gamepad1.dpad_left) strafe = STRAFE_SPEED;
        if (gamepad1.dpad_right) strafe = -STRAFE_SPEED;

        // Face buttons for slow precise movement
        if (gamepad1.y) drive = DRIVE_SPEED * SECONDARY_DILATION;
        if (gamepad1.a) drive = -DRIVE_SPEED * SECONDARY_DILATION;
        if (gamepad1.x) strafe = STRAFE_SPEED * SECONDARY_DILATION;
        if (gamepad1.b) strafe = -STRAFE_SPEED * SECONDARY_DILATION;

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
        if (spindexerState == SpindexerState.OUTTAKE_MODE) {
            telemetry.addLine(String.format("Position: Outtake (%d) - %.2f",
                    outtakePos % 3, OUTTAKE_SLOTS[outtakePos % 3]));
        } else {
            telemetry.addLine(String.format("Position: Intake (%d) - %.2f",
                    intakePos % 3, INTAKE_SLOTS[intakePos % 3]));
        }

        // Transfer telemetry
        telemetry.addLine("\n=== TRANSFER ===");
        telemetry.addLine("State: " + transferState);
        telemetry.addData("Position", transfer.getPosition());

        // Intake telemetry
        telemetry.addLine("\n=== INTAKE ===");
        telemetry.addLine("State: " + intakeState);

        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean shutdown
        flywheel.setVelocity(0);
        intake.setPower(0);
        hoodServo.setPower(0);
    }
}