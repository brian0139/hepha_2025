package org.firstinspires.ftc.teamcode.Stanley;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Aaron.aprilTagV3;

@TeleOp
public class limelighttest extends LinearOpMode {

    private aprilTagV3 aprilTagOperator;

    @Override
    public void runOpMode() throws InterruptedException
    {
        aprilTagOperator=new aprilTagV3(hardwareMap);
        aprilTagOperator.setPipeline(3);
        aprilTagOperator.init();
        waitForStart();
        while (opModeIsActive()) {
            aprilTagOperator.scanOnce();
            telemetry.addData("tx", aprilTagOperator.getYaw());
            telemetry.addData("ty", aprilTagOperator.getPitch());
            telemetry.addData("Botpose", aprilTagOperator.getBotPose());
            telemetry.addData("Tag ID",aprilTagOperator.getTagId());
            telemetry.update();
        }
    }
}