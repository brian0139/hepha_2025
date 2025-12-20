package org.firstinspires.ftc.teamcode.Aaron;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * AprilTag vision subsystem for DECODE
 * - Detects tags on the Obelisk to read motifs
 * - Provides distance, yaw, and simplified motif code (1, 2, 3)
 * - Maintains a list of all detected tags each scan
 *
 * INPUT: Webcam feed (default: "Webcam 1", can be specified in constructor)
 * OUTPUT: Motif code (1, 2, or 3), Tag ID, Distance, Yaw, and list of detections
 */
public class aprilTag {
    private final HardwareMap hardwareMap;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    // List of all detections from the last scan
    private final List<AprilTagDetection> currentDetections = new ArrayList<>();

    // Store the "best" (most relevant) detection
    private AprilTagDetection lastDetection = null;

    // Replace IDs with actual Decode field tag IDs
    private static final int TAG_ID_A = 21; //gpp
    private static final int TAG_ID_B = 22; //pgp
    private static final int TAG_ID_C = 23; //ppg

    // Camera name (defaults to "Webcam 1")
    private String cameraName = "limelight";

    /** Constructor: requires HardwareMap (uses default camera name "Webcam 1") */
    public aprilTag(HardwareMap hwMap) {
        hardwareMap = hwMap;
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder();
        builder.setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(false);
        tagProcessor = builder.build();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, cameraName),
                tagProcessor
        );
    }

    /**
     * Scans for AprilTags once per loop.
     * Updates lastDetection and the list of all detections.
     */
    public void scanOnce() {
        currentDetections.clear();
        currentDetections.addAll(tagProcessor.getDetections());

        // Choose one "best" detection (could refine this later)
        if (!currentDetections.isEmpty()) {
            lastDetection = currentDetections.get(0);
        } else {
            lastDetection = null;
        }
    }

    /** Return the full list of detections from the latest scan */
    public List<AprilTagDetection> getAllDetections() {
        return currentDetections;
    }

    /** Return ID of the last detected tag */
    public int getTagId() {
        return (lastDetection != null) ? lastDetection.id : -1;
    }

    /** Return the most recent detection object (includes ftcPose) */
    public AprilTagDetection getDetection() {
        return lastDetection;
    }

    /** Return distance (m) from camera to tag using Euclidean distance */
    public double getDistance() {
        if (lastDetection == null) return Double.NaN;
        double x = lastDetection.ftcPose.x;
        double y = lastDetection.ftcPose.y;
        double z = lastDetection.ftcPose.z;
        return Math.sqrt(x * x + y * y + z * z);
    }

    /** Return yaw (radians) from camera to tag */
    public double getYaw() {
        return (lastDetection != null) ? lastDetection.ftcPose.yaw : Double.NaN;
    }

    /**
     * Return motif as an integer (1, 2, or 3)
     * - Returns -1 if no tag is detected or unrecognized.
     */
    public int getMotifCode() {
        int id = getTagId();
        if (id == TAG_ID_A) return 1;
        if (id == TAG_ID_B) return 2;
        if (id == TAG_ID_C) return 3;
        return -1;
    }

    /** Clear last detection (optional reset) */
    public void resetDetection() {
        lastDetection = null;
        currentDetections.clear();
    }
}
