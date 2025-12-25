package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class spindexerTestNew extends LinearOpMode{
    CRServo spindexer;
    DcMotor transfer;
    DcMotor intake;

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        transfer=hardwareMap.get(DcMotor.class,"par1");
        intake=hardwareMap.get(DcMotor.class,"intake");
        double spindexerPower=0.5;
        double transferPower=1;
        boolean runtransfer=false;
        boolean runspindexer=false;
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        boolean lasty=false;
        //repeat untill opmode ends
        //TODO:Limit transfer to 0.1<=x<=0.30
        while (opModeIsActive()){
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            if (gamepad1.yWasPressed()) runtransfer=!runtransfer;
            if (gamepad1.xWasPressed()) runspindexer=!runspindexer;
            if (runtransfer){
                transfer.setPower(transferPower);
            }else{
                transfer.setPower(0);
            }
            if (runspindexer){
                spindexer.setPower(spindexerPower);
            }else{
                spindexer.setPower(-gamepad1.left_stick_x);
            }
            telemetry.addData("Running spindexer",runspindexer);
            telemetry.addData("Running transfer",runtransfer);
            telemetry.update();
        }
    }
}