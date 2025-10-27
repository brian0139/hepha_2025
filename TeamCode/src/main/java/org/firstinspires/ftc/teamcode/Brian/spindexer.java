package org.firstinspires.ftc.teamcode.Brian;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Alvin.intake;
//0empty, 1green, 2purple
/* inputs
motif patterns (array)
balls currently in (array)
*/
/* outputs
current spindexer-outtake ball location (int)
*/






@TeleOp(name="spindexer", group="FTC")
public class spindexer extends LinearOpMode {

    // Declare motor variable
    private DcMotor spindexerMotor = null;

    @Override
    public void checkSpindexer() {
        // initialize motor
        spindexerServo = hardwareMap.get(CRServo.class, "spindexerServo");
        spindexerServo.setDirection(CRServo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        intake slotStatus = new intake();
        int[] tmp=slotStatus.slots;

        //pull motif pattern
        //rotate to required motif pattern


            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power", spindexerMotor.getPower());
            telemetry.update();
        }
    }
}