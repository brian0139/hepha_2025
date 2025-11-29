package org.firstinspires.ftc.teamcode.Stanley;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class spindexerTest extends LinearOpMode{
    Servo spindexer;
    Servo transfer;

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        spindexer=hardwareMap.get(Servo.class,"spindexerServo");
        transfer=hardwareMap.get(Servo.class,"transferServo");
        double spindexerpos=0;
        double transferPos=0.3;
        double spindexerDialation=0.001;
        double transferDialation=0.001;
        double[] outtakeslots = {60.0/360, 180.0/360, 300.0/360};
        double[] transferpositions ={0.1,0.3};
        int outtakePos=0;
        int transferListPos=0;
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        boolean lasty=false;
        boolean lastx=false;
        //repeat untill opmode ends
        //TODO:Limit transfer to 0.1<=x<=0.30
        while (opModeIsActive()){
            if (spindexerpos-gamepad1.left_stick_x*spindexerDialation>=0 && spindexerpos-gamepad1.left_stick_x*spindexerDialation<=1){
                spindexerpos-=gamepad1.left_stick_x*spindexerDialation;
            }
            if (transferPos+gamepad1.right_stick_y*transferDialation>=0.1 && transferPos+gamepad1.right_stick_y*transferDialation<=0.30){
                transferPos+=gamepad1.right_stick_y*transferDialation;
            }
            if (gamepad1.y && !lasty){
                lasty=true;
                outtakePos++;
                spindexerpos=outtakeslots[outtakePos%3];
            }else if (!gamepad1.y){
                lasty=false;
            }
            if (gamepad1.x && !lastx){
                lastx=true;
                transferListPos++;
                transferPos= transferpositions[transferListPos%2];
            }else if (!gamepad1.x){
                lastx=false;
            }
            spindexer.setPosition(spindexerpos);
            transfer.setPosition(transferPos);
            telemetry.addData("spindexerPos:",spindexerpos);
            telemetry.addData("transferPos:",transferPos);
            telemetry.addLine("transferListPos:"+transferListPos+"("+ transferpositions[transferListPos%2]+")");
            telemetry.addLine("outtakePos:"+outtakePos+"("+outtakeslots[outtakePos%3]+")");
            telemetry.update();
            
//            if (gamepad1.y){
//                transfer.setPosition(1);
//            }else{
//                transfer.setPosition(0);
//            }
        }
    }
}