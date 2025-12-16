package org.firstinspires.ftc.teamcode.Aaron;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

public class aprilTagV3 {
    private Limelight3A limelight;
    private IMU imu;

    // Replace with actual Decode tag IDs
    private static final int TAG_ID_A=21;
    private static final int TAG_ID_B=22;
    private static final int TAG_ID_C=23;

    private int currentTagId=-1;
    private int currentMotif=-1;
    private double tx=0.0;
    private double ty=0.0;
    private double ta=0.0;
    private Pose3D botPose=null;

    private final List<Integer> motifList=new ArrayList<>();

    public aprilTagV3(HardwareMap hwMap){
        limelight=hwMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(9); //TODO: AprilTag pipeline CHANGE THIS CUZ DK YET

        imu = hwMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot=new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    public void init(){
        limelight.start();
    }

    // Scan for AprilTags once per loop
    public void scanOnce(){
        motifList.clear();

        // Update Limelight with robot orientation for better localization
        YawPitchRollAngles orientation=imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        // Get latest results from Limelight
        LLResult llResult=limelight.getLatestResult();

        if (llResult!=null && llResult.isValid()){
            // Get AprilTag results
            List<LLResultTypes.FiducialResult> fiducials=llResult.getFiducialResults();

            if (fiducials!=null && !fiducials.isEmpty()){
                // Get the first detected AprilTag
                LLResultTypes.FiducialResult fiducial=fiducials.get(0);

                // Get the AprilTag ID
                currentTagId=(int)fiducial.getFiducialId();

                // Map to motif code
                currentMotif=mapTagToMotif(currentTagId);

                if (currentMotif!=-1){
                    motifList.add(currentMotif);
                }

                // Store targeting data
                tx=llResult.getTx();
                ty=llResult.getTy();
                ta=llResult.getTa();

                // Get robot pose
                botPose=llResult.getBotpose_MT2();
            } else {
                // No AprilTags detected
                currentTagId=-1;
                currentMotif=-1;
            }
        } else {
            // No valid result
            currentTagId=-1;
            currentMotif=-1;
        }
    }

    // Check if have valid target
    public boolean hasValidTarget(){
        return currentTagId!=-1;
    }

    // Returns all detected motif codes
    public List<Integer> getAllDetections(){
        return motifList;
    }

    // Returns the raw AprilTag ID
    public int getTagId(){
        return currentTagId;
    }

    // Returns primary motif code
    public int getMotifCode(){
        return currentMotif;
    }

    // Returns yaw (tx) in degrees
    public double getYaw(){
        return tx;
    }

    // Returns pitch (ty) in degrees
    public double getPitch(){
        return ty;
    }

    // Returns target area percentage
    public double getTargetArea(){
        return ta;
    }

    // Returns robot pose from Limelight
    public Pose3D getBotPose(){
        return botPose;
    }

    // Returns distance to target
    public double getDistance(){
        if (botPose==null){
            return Double.NaN;
        }
        return botPose.getPosition().z;
    }

    // Get X position from botpose
    public double getX(){
        if (botPose==null) {
            return Double.NaN;
        }
        return botPose.getPosition().x;
    }

    // Get Y position from botpose
    public double getY(){
        if (botPose==null){
            return Double.NaN;
        }
        return botPose.getPosition().y;
    }

    // Map raw AprilTag IDs to Decode motif numbers */
    private int mapTagToMotif(int id){
        if (id == TAG_ID_A) return 1;
        if (id == TAG_ID_B) return 2;
        if (id == TAG_ID_C) return 3;
        return -1; // Unknown tag
    }

    // Reset stored detections
    public void resetDetection(){
        currentTagId=-1;
        currentMotif=-1;
        motifList.clear();
    }

    // Stop Limelight when done
    public void stop(){
        limelight.stop();
    }

    // Switch pipeline
    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
}