package org.firstinspires.ftc.teamcode.Aaron;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
@Autonomous
public class aprilTagV2 extends OpMode{
    private Limelight3A limelight;
    private IMU imu;
    private static final int TAG_ID_A=21;
    private static final int TAG_ID_B=22;
    private static final int TAG_ID_C=23;
    private int currentTagId=-1;
    private int currentMotif=-1;

    @Override
    public void init(){
        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(3); // TODO:AprilTag pipeline CHANGE IF NEEDED CUZ IDK WHAT IS

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        telemetry.addData("Status","Initialized");
        telemetry.addData("Pipeline","3 (AprilTag)");
        telemetry.update();
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        // get latest results from Limelight
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            // get apriTag results
            List<LLResultTypes.FiducialResult> fiducials=llResult.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()){
                LLResultTypes.FiducialResult fiducial=fiducials.get(0);
                currentTagId=(int)fiducial.getFiducialId();
                currentMotif=mapTagToMotif(currentTagId);

                Pose3D botPose=llResult.getBotpose_MT2();

                // info stuff
                telemetry.addData("=== DETECTION ===","");
                telemetry.addData("AprilTag ID",currentTagId);
                telemetry.addData("Motif Code",currentMotif!=-1 ? currentMotif:"Unknown");
                telemetry.addData("","");

                telemetry.addData("=== TARGETING ===","");
                telemetry.addData("Tx (Yaw)","%.2f°",llResult.getTx());
                telemetry.addData("Ty (Pitch)","%.2f°",llResult.getTy());
                telemetry.addData("Ta (Area)","%.2f%%",llResult.getTa());
                telemetry.addData("","");

                // Display position if available
                if (botPose != null) {
                    telemetry.addData("=== POSITION ===","");
                    telemetry.addData("X","%.2f",botPose.getPosition().x);
                    telemetry.addData("Y","%.2f",botPose.getPosition().y);
                    telemetry.addData("Z (Distance)","%.2f",botPose.getPosition().z);
                }

                // Show number of tags detected
                telemetry.addData("","");
                telemetry.addData("Total Tags",fiducials.size());

            }
            else{
                // No AprilTags detected
                currentTagId=-1;
                currentMotif=-1;
                telemetry.addData("Status","No AprilTags Detected");
            }
        }
        else{
            // No valid result
            currentTagId=-1;
            currentMotif=-1;
            telemetry.addData("Status","No Valid Result");
        }
        telemetry.update();
    }

    @Override
    public void stop(){
        limelight.stop();
    }
    private int mapTagToMotif(int id){
        if (id == TAG_ID_A) return 1;
        if (id == TAG_ID_B) return 2;
        if (id == TAG_ID_C) return 3;
        return -1;
    }

    // get tag id
    public int getTagId(){
        return currentTagId;
    }

    // get motif code
    public int getMotifCode(){
        return currentMotif;
    }
}