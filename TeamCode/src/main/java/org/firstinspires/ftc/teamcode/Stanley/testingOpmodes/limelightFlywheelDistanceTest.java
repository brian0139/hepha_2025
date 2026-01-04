package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3;

@TeleOp
public class limelightFlywheelDistanceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        outtakeV3 outtake = new outtakeV3(hardwareMap, "Red", true);
        boolean autoSpeedEnabled = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) autoSpeedEnabled = !autoSpeedEnabled;

            double setpoint = autoSpeedEnabled ? outtake.updateFlywheelSpeedFromTagDistance() : 0.0;
            if (!autoSpeedEnabled) outtake.spin_flywheel(0.0, 25);

            telemetry.addData("Auto Speed", autoSpeedEnabled);
            telemetry.addData("Has Target", outtake.apriltag != null && outtake.apriltag.hasValidTarget());
            telemetry.addData("Distance M (last)", outtake.lastTagDistanceM);
            telemetry.addData("Setpoint (tps)", setpoint);
            telemetry.addData("FlywheelR Vel (tps)", outtake.flywheelDriveR.getVelocity());
            telemetry.update();
        }
    }
}

