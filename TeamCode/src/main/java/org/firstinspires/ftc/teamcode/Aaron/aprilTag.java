package org.firstinspires.ftc.teamcode.Aaron;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * AprilTag vision subsystem for DECODE
 * - Detects tags on the Obelisk to read motifs
 * - Can provide relative position/angle for aiming
 */
public class aprilTag {
    private final HardwareMap hardwareMap;   // <-- marked final
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private AprilTagDetection lastDetection = null;

    // Motif mapping (replace IDs with the actual ones from the DECODE field guide)
    public enum Motif { MOTIF_A, MOTIF_B, MOTIF_C, UNKNOWN }
    private static final int TAG_ID_A = 1;
    private static final int TAG_ID_B = 2;
    private static final int TAG_ID_C = 3;

    /** Constructor: requires HardwareMap */
    public aprilTag(HardwareMap hwMap) {
        hardwareMap = hwMap;   // now initialized safely
    }


    /** Initialize AprilTag processor and VisionPortal */
    public void init() {
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder();
        builder.setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(false);
        tagProcessor = builder.build();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                tagProcessor
        );
    }

    /** Scan once per loop and update detection */
    public void scanOnce() {
        for (AprilTagDetection det : tagProcessor.getDetections()) {
            // pick "best" detection — customize if needed
            if (lastDetection == null || det.ftcPose.y < lastDetection.ftcPose.y) {
                lastDetection = det;
            }
        }
    }

    /** Return ID of the last detected tag */
    public int getTagId() {
        return (lastDetection != null) ? lastDetection.id : -1;
    }

    /** Return the full AprilTagDetection (includes ftcPose) */
    public AprilTagDetection getDetection() {
        return lastDetection;
    }

    /** Return distance (m) from camera to tag (Euclidean) */
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

    /** Return motif based on tag ID */
    public Motif getCurrentMotif() {
        return mapIdToMotif(getTagId());
    }

    private Motif mapIdToMotif(int id) {
        if (id == TAG_ID_A) return Motif.MOTIF_A;
        if (id == TAG_ID_B) return Motif.MOTIF_B;
        if (id == TAG_ID_C) return Motif.MOTIF_C;
        return Motif.UNKNOWN;
    }

    /** Clear last detection */
    public void resetDetection() {
        lastDetection = null;
    }
}
