package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * FTC helper for Limelight3A including AprilTag support.
 */
public class LimelightHelpers {

    private final Limelight3A limelight;

    /** Constructor */
    public LimelightHelpers(HardwareMap hardwareMap, String name) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    /** Returns the latest LLResult */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /** Returns true if Limelight sees a valid target */
    public boolean hasTarget() {
        LLResult result = getLatestResult();
        return result != null && result.isValid();
    }

    /** Horizontal offset from crosshair to target (degrees) */
    public double getTX() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTx() : 0.0;
    }

    /** Vertical offset from crosshair to target (degrees) */
    public double getTY() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0.0;
    }

    /** Target area (% of image) */
    public double getTA() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTa() : 0.0;
    }

    /** Switch to a different pipeline index (0-9) */
    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    /** Returns the first detected AprilTag ID (fiducial ID) */
    public int getFiducialID() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                return fiducials.get(0).getFiducialId();
            }
        }
        return -1;
    }

    /**
     * Returns Limelight bot pose as double[6]: x, y, z, roll, pitch, yaw
     * Pose is in FTC coordinate frame; roll/pitch/yaw in degrees
     */
    /**
     * Returns Limelight bot pose as double[6]: x, y, z, roll, pitch, yaw
     * Pose is in FTC coordinate frame; roll/pitch/yaw in degrees
     */
    /**
     * Returns Limelight bot pose as double[6]: x, y, z, roll, pitch, yaw
     * Pose is in FTC coordinate frame; roll/pitch/yaw in degrees
     */
    public double[] getBotPose() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose();
            if (pose != null) {
                // Access translation using getPosition()
                double x = pose.getPosition().x;
                double y = pose.getPosition().y;
                double z = pose.getPosition().z;

                // Access orientation using getRoll, getPitch, and getYaw methods
                // These require an AngleUnit parameter
                double roll = pose.getOrientation().getRoll(AngleUnit.DEGREES);
                double pitch = pose.getOrientation().getPitch(AngleUnit.DEGREES);
                double yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);

                return new double[]{x, y, z, roll, pitch, yaw};
            }
        }
        return null;
    }


}