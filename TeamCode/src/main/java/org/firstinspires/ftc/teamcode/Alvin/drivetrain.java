package org.firstinspires.ftc.teamcode.Stanley.finalizedOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class drivetrain extends LinearOpMode {

    // ==================== HARDWARE ====================
    // drivetrain motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // other motors
    private DcMotorEx flywheel, intake;
    private DcMotor transfer;

    // servos
    private CRServo hoodServo;
    private Servo spindexer;

    // ==================== STATE MACHINE ENUMS ====================
    private enum FlywheelState { OFF, ON }
    private enum TransferState { STOPPED, RUNNING }
    private enum SpindexerState { POS_0, POS_1 }
    private enum HoodState { STOPPED, LEFT, RIGHT }

    // ==================== STATE ====================
    private FlywheelState flywheelState = FlywheelState.OFF;
    private TransferState transferState = TransferState.STOPPED;
    private SpindexerState spindexerState = SpindexerState.POS_1;
    private HoodState hoodState = HoodState.STOPPED;

    // ==================== CONFIG ====================
    private static final int FLYWHEEL_SENSITIVITY = 10;
    private static final double HOOD_SPEED = 0.5;

    private static final double DRIVE_SPEED = 0.7;
    private static final double STRAFE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.5;

    private static final double[] SPINDEXER_POSITIONS = {0.0, 0.75};

    // ==================== WORKING VARS ====================
    private int flywheelSpeed = 2000;
    private int targetSpeed = 0;

    // edge-detect
    private boolean prevY = false;
    private boolean prevX = false;
    private boolean prevRB = false;

    @Override
    public void runOpMode() {
        // init drivetrain
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // other motors
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "par1");

        // servos
        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");
        spindexer = hardwareMap.get(Servo.class, "spindexerServo");

        // initial positions
        spindexer.setPosition(SPINDEXER_POSITIONS[1]);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1) update state machines (transitions)
            updateFlywheelSM();
            updateSpindexerSM();
            updateTransferSM();
            updateHoodSM();

            // 2) continuous controls (intake + drivetrain)
            updateIntake();
            updateDrivetrain();

            // 3) telemetry
            updateTelemetry();

            // 4) store edges
            prevY = gamepad1.y;
            prevX = gamepad1.x;
            prevRB = gamepad1.right_bumper;
        }
    }

    // ==================== FLYWHEEL SM ====================
    private void updateFlywheelSM() {
        // speed adjust (dpad up/down)
        if (gamepad1.dpad_up) {
            flywheelSpeed += FLYWHEEL_SENSITIVITY;
        } else if (gamepad1.dpad_down) {
            flywheelSpeed -= FLYWHEEL_SENSITIVITY;
            if (flywheelSpeed < 0) flywheelSpeed = 0;
        }

        // toggle (Y edge)
        boolean yEdge = gamepad1.y && !prevY;
        if (yEdge) {
            flywheelState = (flywheelState == FlywheelState.OFF) ? FlywheelState.ON : FlywheelState.OFF;
        }

        // action
        if (flywheelState == FlywheelState.ON) {
            targetSpeed = flywheelSpeed;
            flywheel.setVelocity(targetSpeed);
        } else {
            targetSpeed = 0;
            flywheel.setVelocity(0);
        }
    }

    // ==================== SPINDEXER SM ====================
    private void updateSpindexerSM() {
        // toggle between two positions with right bumper edge
        boolean rbEdge = gamepad1.right_bumper && !prevRB;
        if (rbEdge) {
            spindexerState = (spindexerState == SpindexerState.POS_0) ? SpindexerState.POS_1 : SpindexerState.POS_0;
        }

        // action
        int idx = (spindexerState == SpindexerState.POS_0) ? 0 : 1;
        spindexer.setPosition(SPINDEXER_POSITIONS[idx]);
    }

    // ==================== TRANSFER SM ====================
    private void updateTransferSM() {
        // toggle (X edge)
        boolean xEdge = gamepad1.x && !prevX;
        if (xEdge) {
            transferState = (transferState == TransferState.STOPPED) ? TransferState.RUNNING : TransferState.STOPPED;
        }

        // action
        transfer.setPower(transferState == TransferState.RUNNING ? 1.0 : 0.0);
    }

    // ==================== HOOD SM ====================
    private void updateHoodSM() {
        // dpad right/left = move, else stop
        if (gamepad1.dpad_right) {
            hoodState = HoodState.RIGHT;
        } else if (gamepad1.dpad_left) {
            hoodState = HoodState.LEFT;
        } else {
            hoodState = HoodState.STOPPED;
        }

        // action
        switch (hoodState) {
            case RIGHT: hoodServo.setPower(HOOD_SPEED); break;
            case LEFT:  hoodServo.setPower(-HOOD_SPEED); break;
            default:    hoodServo.setPower(0); break;
        }
    }

    // ==================== INTAKE (continuous) ====================
    private void updateIntake() {
        double p = gamepad1.right_trigger - gamepad1.left_trigger;
        intake.setPower(p);
    }

    // ==================== DRIVETRAIN (continuous) ====================
    private void updateDrivetrain() {
        double drive  = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = -gamepad1.left_stick_x * STRAFE_SPEED;
        double twist  = -gamepad1.right_stick_x * TURN_SPEED;

        double[] speeds = {
                (drive - strafe - twist), // leftFront
                (drive + strafe + twist), // rightFront
                (drive + strafe - twist), // leftBack
                (drive - strafe + twist)  // rightBack
        };

        double max = Math.abs(speeds[0]);
        for (int i = 1; i < speeds.length; i++) {
            max = Math.max(max, Math.abs(speeds[i]));
        }
        if (max > 1.0) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        leftFront.setPower(speeds[0]);
        rightFront.setPower(speeds[1]);
        leftBack.setPower(speeds[2]);
        rightBack.setPower(speeds[3]);
    }

    // ==================== TELEMETRY ====================
    private void updateTelemetry() {
        telemetry.addLine("=== FLYWHEEL ===");
        telemetry.addData("State", flywheelState);
        telemetry.addData("Set Speed (ticks/s)", flywheelSpeed);
        telemetry.addData("Target (ticks/s)", targetSpeed);
        telemetry.addData("Actual (ticks/s)", flywheel.getVelocity());

        telemetry.addLine("\n=== SPINDEXER ===");
        telemetry.addData("State", spindexerState);
        telemetry.addData("Pos", spindexer.getPosition());

        telemetry.addLine("\n=== TRANSFER ===");
        telemetry.addData("State", transferState);
        telemetry.addData("Power", transfer.getPower());

        telemetry.addLine("\n=== HOOD ===");
        telemetry.addData("State", hoodState);

        telemetry.update();
    }
}
