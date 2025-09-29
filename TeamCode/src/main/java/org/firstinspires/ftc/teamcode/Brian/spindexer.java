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

import org.firstinspires.ftc.teamcode.Alvin.intake;


@TeleOp(name="spindexer", group="FTC")
public class spindexer extends LinearOpMode {

    // Declare motor variable
    private DcMotor spindexerMotor = null;

    @Override
    public void checkSpindexer() {
        // initialize motor
        spindexerMotor = hardwareMap.get(DcMotor.class, "spindexerMotor");
        spindexerMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        intake slotStatus = new intake();
        int[] tmp=slotStatus.slots;


            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power", spindexerMotor.getPower());
            telemetry.update();
        }
    }
}