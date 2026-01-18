package org.firstinspires.ftc.teamcode.Alvin;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alvin.intake;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3;

@Autonomous(name = "DECODE Auto Red - Fast Sort", group = "Autonomous")
public class DecodeAutoRed extends LinearOpMode {

    // Starting pose - Red side near samples
    private final Pose2d START_POSE = new Pose2d(-48, 60, Math.toRadians(270));

    // Field positions for Red alliance
    private final Vector2d RED_CLASSIFIER = new Vector2d(-48, 48);

    // Sample positions - 3 rows of 4 samples each
    private final Vector2d[] SAMPLE_POSITIONS = {
            // Row 1
            new Vector2d(-60, 40),
            new Vector2d(-52, 40),
            new Vector2d(-44, 40),
            new Vector2d(-36, 40),
            // Row 2
            new Vector2d(-60, 32),
            new Vector2d(-52, 32),
            new Vector2d(-44, 32),
            new Vector2d(-36, 32),
            // Row 3
            new Vector2d(-60, 24),
            new Vector2d(-52, 24),
            new Vector2d(-44, 24),
            new Vector2d(-36, 24)
    };

    private MecanumDrive drive;
    private intake intakeSystem;
    private spindexerColor spindexer;
    private outtakeV3 outtake;

    @Override
    public void runOpMode() {
        // Initialize hardware
        drive = new MecanumDrive(hardwareMap, START_POSE);
        intakeSystem = new intake(hardwareMap, "intake", "intakeSensor");
        spindexer = new spindexerColor(
                hardwareMap.crservo.get("spindexerServo"),
                hardwareMap.dcMotor.get("intake"),
                hardwareMap
        );
        outtake = new outtakeV3(hardwareMap, "Red", false, drive);

        telemetry.addData("Status", "Initialized - Red Alliance");
        telemetry.addData("Mission", "Fast 12 Ball Sort");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Cycle 1: Samples 0-2
        driveToPose(SAMPLE_POSITIONS[0], Math.toRadians(270));
        intakeBall();

        driveToPose(SAMPLE_POSITIONS[1], Math.toRadians(270));
        intakeBall();

        driveToPose(SAMPLE_POSITIONS[2], Math.toRadians(270));
        intakeBall();

        driveToPose(RED_CLASSIFIER, Math.toRadians(180));
        scoreThreeBalls();

        // Cycle 2: Samples 3-5
        driveToPose(SAMPLE_POSITIONS[3], Math.toRadians(270));
        intakeBall();

        driveToPose(SAMPLE_POSITIONS[4], Math.toRadians(270));
        intakeBall();

        driveToPose(SAMPLE_POSITIONS[5], Math.toRadians(270));
        intakeBall();

        driveToPose(RED_CLASSIFIER, Math.toRadians(180));
        scoreThreeBalls();

        // Cycle 3: Samples 6-8
        driveToPose(SAMPLE_POSITIONS[6], Math.toRadians(270));
        intakeBall();

        driveToPose(SAMPLE_POSITIONS[7], Math.toRadians(270));
        intakeBall();

        driveToPose(SAMPLE_POSITIONS[8], Math.toRadians(270));
        intakeBall();

        driveToPose(RED_CLASSIFIER, Math.toRadians(180));
        scoreThreeBalls();

        // Cycle 4: Samples 9-11
        driveToPose(SAMPLE_POSITIONS[9], Math.toRadians(270));
        intakeBall();

        driveToPose(SAMPLE_POSITIONS[10], Math.toRadians(270));
        intakeBall();

        driveToPose(SAMPLE_POSITIONS[11], Math.toRadians(270));
        intakeBall();

        driveToPose(RED_CLASSIFIER, Math.toRadians(180));
        scoreThreeBalls();

        // Park
        driveToPose(new Vector2d(-48, 12), Math.toRadians(270));
    }

    /**
     * Drive to a position using Road Runner
     */
    private void driveToPose(Vector2d position, double heading) {
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(position, heading)
                        .build()
        );
    }

    /**
     * Intake a ball and load into spindexer
     */
    private void intakeBall() {
        // Run intake until ball detected
        while (!intakeSystem.intakeUntilPixel() && opModeIsActive()) {
            telemetry.addData("Status", "Intaking...");
            telemetry.update();
        }

        // Spin spindexer to next empty slot
        spindexer.initSpin();
        while (!spindexer.spinToIntake() && opModeIsActive()) {
            telemetry.addData("Status", "Loading spindexer...");
            telemetry.update();
        }

        // Record color
        spindexer.getColor();

        telemetry.addData("Status", "Ball loaded!");
        telemetry.update();
        sleep(200);
    }

    /**
     * Score 3 balls from spindexer in sorted order
     */
    private void scoreThreeBalls() {
        // For each of the 3 balls in spindexer
        for (int i = 0; i < 3; i++) {
            // Get motif for current ball
            int motif = spindexer.dummyMotif[i];

            // Spin spindexer to position this ball at outtake
            while (!spindexer.spinToMotif(i) && opModeIsActive()) {
                telemetry.addData("Status", "Positioning ball " + (i+1));
                telemetry.update();
            }

            // Release ball through gate
            outtake.transferUp();
            sleep(500);
            outtake.transferDown();
            sleep(300);

            telemetry.addData("Status", "Ball " + (i+1) + " scored - Motif " + motif);
            telemetry.update();
        }

        // Reset spindexer state
        spindexer.detectioncnt = -1;
    }
}