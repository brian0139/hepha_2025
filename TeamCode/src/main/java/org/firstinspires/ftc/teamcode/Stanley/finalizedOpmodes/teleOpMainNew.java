package org.firstinspires.ftc.teamcode.Stanley.finalizedOpmodes;

import com.qualcomm.robotcore.hardware.AnalogInput;
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
public class teleOpMainNew extends OpMode {

    // ==================== STATE MACHINE ENUMS ====================
    enum FlywheelState {
        IDLE,
        SPINNING,
        STOPPED
    }

    enum TransferState {
        UP,
        STOPPED
    }

    enum IntakeState {
        STOPPED,
        INTAKING,
        OUTTAKING
    }

    // ==================== HARDWARE DECLARATION ====================
    // Drivetrain motors
    DcMotor leftFront = null;
    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor rightBack = null;

    // Other motors
    DcMotorEx flywheel = null;
    DcMotorEx flywheelR = null;
    DcMotor intake = null;

    // Servos
    CRServo hoodServo = null;
    Servo spindexer = null;
    Servo transfer = null;

    //Analog Input
    AnalogInput spindexerAnalog = null;

    // ==================== STATE VARIABLES ====================
    FlywheelState flywheelState = FlywheelState.IDLE;
    TransferState transferState = TransferState.STOPPED;
    IntakeState intakeState = IntakeState.STOPPED;

    // ==================== CONFIGURATION ====================
    static final double FLYWHEEL_SENSITIVITY = 5;
    static final double FLYWHEEL_EPSILON = 10;
    static final double DRIVE_SPEED = 0.7;
    static final double STRAFE_SPEED = 0.5;
    static final double TWIST_SPEED = 0.5;
    static final double SECONDARY_DILATION = 0.25;

    static final double[] TRANSFER_POWERS = {1, 0};
    static final int TRANSFER_DOWN = 1;
    static final int TRANSFER_UP = 0;

    // ==================== WORKING VARIABLES ====================
    int flywheelSpeed = 2000;
    int targetSpeed = 0;

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
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelR = hardwareMap.get(DcMotorEx.class,"flywheelR");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize servos
        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");
        spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        transfer = hardwareMap.get(Servo.class, "transferServo");

        // Initialize analog input
        spindexerAnalog = hardwareMap.get(AnalogInput.class,"spindexerAnalog");

        // Set initial positions
        transfer.setPosition(TRANSFER_POWERS[TRANSFER_DOWN]);
        transferState = TransferState.STOPPED;

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
    }

    // ==================== FLYWHEEL STATE MACHINE ====================
    void updateFlywheelStateMachine() {
        // Adjust speed with right stick
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            int speedChange = (int)(gamepad2.right_stick_y * FLYWHEEL_SENSITIVITY);
            if (flywheelSpeed - speedChange >= 0) {
                flywheelSpeed -= speedChange;
                if (flywheelState == FlywheelState.SPINNING) {
                    flywheelState = FlywheelState.STOPPED;
                }
            } else {
                flywheelSpeed = 0;
            }
        }

        // Toggle flywheel on/off with Y button
        if (gamepad2.yWasPressed()) {
            switch (flywheelState) {
                case STOPPED:

                case IDLE:
                    flywheelState = FlywheelState.SPINNING;
                    targetSpeed = flywheelSpeed;
                    flywheel.setVelocity(targetSpeed);
                    break;
                case SPINNING:
                    flywheelState = FlywheelState.STOPPED;
                    targetSpeed = 0;
                    flywheel.setVelocity(0);
                    break;
            }
        }
    }

    // ==================== SPINDEXER STATE MACHINE ====================
    void updateSpindexerStateMachine() {

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
    void updateTransferStateMachine() {
        // Only allow transfer control in outtake mode
        if (spindexerState != SpindexerState.OUTTAKE_MODE) {
            return;
        }

        switch (transferState) {
            case DOWN:
                if (gamepad2.x) {
                    transferState = TransferState.MOVING;
                    transfer.setPosition(TRANSFER_POWERS[TRANSFER_UP]);
                    transferState = TransferState.UP;
                }
                break;

            case UP:
                if (!gamepad2.x) {
                    transferState = TransferState.MOVING;
                    transfer.setPosition(TRANSFER_POWERS[TRANSFER_DOWN]);
                    transferState = TransferState.DOWN;
                }
                break;

            case MOVING:
                // Servo movement is instant, so this state is brief
                break;
        }
    }

    // ==================== INTAKE STATE MACHINE ====================
    void updateIntakeStateMachine() {
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
    void updateHoodControl() {
        hoodServo.setPower(-gamepad2.left_stick_y);
    }

    // ==================== DRIVETRAIN CONTROL ====================
    void updateDrivetrain() {
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
    void updateTelemetry() {


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