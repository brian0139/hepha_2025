package org.firstinspires.ftc.teamcode.Alvin;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alvin.intake;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3;

@Autonomous(name = "DECODE Auto Red - 12 Ball Sort", group = "Autonomous")
public class DecodeAutoRed extends LinearOpMode {

    // Starting pose - Red side against wall near classifier
    private final Pose2d START_POSE = new Pose2d(-36, 63, Math.toRadians(270));

    // Field positions for Red alliance
    private final Vector2d RED_CLASSIFIER = new Vector2d(-36, 52);
    private final Vector2d[] SAMPLE_POSITIONS = {
            new Vector2d(-48, 48), // Sample 1
            new Vector2d(-58, 48), // Sample 2
            new Vector2d(-48, 36), // Sample 3
            new Vector2d(-58, 36), // Sample 4
            new Vector2d(-48, 24), // Sample 5
            new Vector2d(-58, 24), // Sample 6
            new Vector2d(-48, 12), // Sample 7
            new Vector2d(-58, 12), // Sample 8
            new Vector2d(-48, 0),  // Sample 9
            new Vector2d(-58, 0),  // Sample 10
            new Vector2d(-48, -12),// Sample 11
            new Vector2d(-58, -12) // Sample 12
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
        telemetry.addData("Mission", "12 Ball Sort - 4 Cycles");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Run the autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        // Cycle 1: Collect 3 balls
                        collectThreeBalls(0),
                        scoreAtClassifier(1),

                        // Cycle 2: Collect 3 balls
                        collectThreeBalls(3),
                        scoreAtClassifier(2),

                        // Cycle 3: Collect 3 balls
                        collectThreeBalls(6),
                        scoreAtClassifier(3),

                        // Cycle 4: Collect 3 balls
                        collectThreeBalls(9),
                        scoreAtClassifier(4),

                        // Park
                        parkRobot()
                )
        );
    }

    /**
     * Collect 3 balls from the field
     * @param startIndex Starting index in SAMPLE_POSITIONS array
     */
    private Action collectThreeBalls(int startIndex) {
        return new SequentialAction(
                // Collect ball 1
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(SAMPLE_POSITIONS[startIndex], Math.toRadians(270))
                        .build(),
                intakeBallAction(),

                // Collect ball 2
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(SAMPLE_POSITIONS[startIndex + 1], Math.toRadians(270))
                        .build(),
                intakeBallAction(),

                // Collect ball 3
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(SAMPLE_POSITIONS[startIndex + 2], Math.toRadians(270))
                        .build(),
                intakeBallAction()
        );
    }

    /**
     * Score sorted balls at the classifier
     * @param cycleNumber Which cycle (1-4)
     */
    private Action scoreAtClassifier(int cycleNumber) {
        return new SequentialAction(
                // Drive to classifier
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(RED_CLASSIFIER, Math.toRadians(180))
                        .build(),

                // Score first ball
                sortAndScoreAction(),

                // Score second ball
                sortAndScoreAction(),

                // Score third ball
                sortAndScoreAction()
        );
    }

    /**
     * Intake a single ball
     */
    private Action intakeBallAction() {
        return telemetryPacket -> {
            // Run intake until ball detected
            if (intakeSystem.intakeUntilPixel()) {
                // Ball collected, spin spindexer to next empty slot
                if (spindexer.spinToIntake()) {
                    telemetry.addData("Intake", "Ball collected");
                    telemetry.update();
                    return false; // Action complete
                }
            }
            return true; // Keep running
        };
    }

    /**
     * Sort ball using spindexer and score in classifier
     */
    private Action sortAndScoreAction() {
        return telemetryPacket -> {
            // Get current motif pattern from AprilTag detection
            // For now using dummy motif pattern
            int currentMotif = spindexer.dummyMotif[spindexer.currentSlot];

            // Spin spindexer to position ball at outtake
            if (spindexer.spinToMotif(currentMotif)) {
                // Open gate to release ball
                outtake.transferUp();
                sleep(500); // Wait for ball to exit
                outtake.transferDown();

                telemetry.addData("Scoring", "Ball scored - Motif " + currentMotif);
                telemetry.update();
                return false; // Action complete
            }
            return true; // Keep running
        };
    }

    /**
     * Park robot at end of autonomous
     */
    private Action parkRobot() {
        return drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-36, 12), Math.toRadians(270))
                .build();
    }
}