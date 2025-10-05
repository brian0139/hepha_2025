package org.firstinspires.ftc.teamcode.Brian;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


@TeleOp(name="spindexer", group="FTC")
public class spindexer extends LinearOpMode {
    public void checkStatus() {
        if (gamepad1.a) {
            for (int slot = 0; slot <3; slot++) {
                if (status[slot] == )
            }

        }
    }

    // Declare motor variable
    private DcMotor spindexerMotor = null;

    @Override
    public void runOpMode() {
        // initialize motor
        spindexerMotor = hardwareMap.get(DcMotor.class, "spindexerMotor");
        spindexerMotor.setDirection(DcMotor.Direction.FORWARD);

        int[] status = {0,0,0};



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                spindexerMotor.setPower(1.0);
            } else {
                spindexerMotor.setPower(0.0);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power", spindexerMotor.getPower());
            telemetry.update();
        }
    }
}
