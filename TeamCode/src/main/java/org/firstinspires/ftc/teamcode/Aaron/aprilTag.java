package org.firstinspires.ftc.teamcode.Aaron;

import org.firstinspires.ftc.teamcode.LimelightHelpers;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;

public class aprilTag {

    private static final String LIMELIGHT_TABLE = "limelight";

    // Replace with actual Decode tag IDs
    private static final int TAG_ID_A = 1;
    private static final int TAG_ID_B = 2;
    private static final int TAG_ID_C = 3;

    private int lastTagId = -1;

    private final List<Integer> motifList = new ArrayList<>();

    private final LimelightHelpers limelight;

    /** Constructor */
    public aprilTag(HardwareMap hwMap) {
        limelight = new LimelightHelpers(hwMap, LIMELIGHT_TABLE);
    }

    /** Optional init method */
    public void init() {
        // No-op for Limelight
    }

    /** Scan Limelight once per loop */
    public void scanOnce() {
        motifList.clear();
        lastTagId = limelight.getFiducialID();  // update class field
        int motif = mapTagToMotif(lastTagId);
        if (motif != -1) {
            motifList.add(motif);
        }
    }

    /** Returns all detected motif codes */
    public List<Integer> getAllDetections() {
        return motifList;
    }

    /** Returns the raw AprilTag ID */
    public int getTagId() {
        return lastTagId;
    }

    /** Returns primary motif code */
    public int getMotifCode() {
        if (motifList.isEmpty()) return -1;
        return motifList.get(0);
    }

    /** Returns yaw (tx) in degrees */
    public double getYaw() {
        return limelight.getTX();
    }

    /** Returns approximate distance from Limelight botpose */
    public double getDistance() {
        double[] pose = limelight.getBotPose();
        if (pose == null) return Double.NaN;
        double x = pose[0], y = pose[1], z = pose[2];
        return Math.sqrt(x*x + y*y + z*z);
    }

    /** Map raw AprilTag IDs to Decode motif numbers */
    private int mapTagToMotif(int id) {
        if (id == TAG_ID_A) return 1;
        if (id == TAG_ID_B) return 2;
        if (id == TAG_ID_C) return 3;
        return -1;
    }

    /** Reset stored detections */
    public void resetDetection() {
        lastTagId = -1;
        motifList.clear();
    }
}