package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class hoodtest extends LinearOpMode{
    CRServo tmpservo;
    DcMotorEx tmpmotor;
    Servo spindexer;
    Servo transfer;
    double power=0;
    double motorSpeed =0;
    double expectedSpeed =0;
    double motorDialation=10;
    double spindexerpos=0;
    double spindexerDialation=0.001;
    double transferDialation=0.003;
    //double[] outtakeslots = {60.0/360, 180.0/360, 300.0/360};
    double[] outtakeslots = {0.26,0.65,1};
    double[] transferpositions ={0.6,0.9};
    double transferPos=transferpositions[1];
    int outtakePos=0;
    int transferListPos=0;
    //main loop
    @Override
    public void runOpMode() {
        tmpservo=hardwareMap.crservo.get("hoodServo");
        tmpmotor= (DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        tmpmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexer=hardwareMap.get(Servo.class,"spindexerServo");
        transfer=hardwareMap.get(Servo.class,"transferServo");
        waitForStart();
        while (opModeIsActive()){
            power=-gamepad1.right_stick_x;
            if (motorSpeed -gamepad1.right_stick_y*motorDialation>=0){
                motorSpeed -=gamepad1.right_stick_y*motorDialation;
            }
            if (gamepad1.yWasPressed()) {
                tmpmotor.setVelocity(motorSpeed);
                expectedSpeed = motorSpeed;
            }
            if (gamepad1.xWasPressed()) {
                tmpmotor.setVelocity(0);
                expectedSpeed = 0;
            }
            tmpservo.setPower(power);
            if (spindexerpos-gamepad1.left_stick_x*spindexerDialation>=0 && spindexerpos-gamepad1.left_stick_x*spindexerDialation<=0.75){
                spindexerpos-=gamepad1.left_stick_x*spindexerDialation;
            }
            if (transferPos+gamepad1.left_stick_y*transferDialation>=0.6 && transferPos+gamepad1.left_stick_y*transferDialation<=0.9){
                transferPos+=gamepad1.left_stick_y*transferDialation;
            }
            if (gamepad1.aWasPressed()){
                outtakePos++;
                spindexerpos=outtakeslots[outtakePos%3];
            }
            if (gamepad1.bWasPressed()){
                transferListPos++;
                transferPos= transferpositions[transferListPos%2];
            }
            spindexer.setPosition(spindexerpos);
            transfer.setPosition(transferPos);
            telemetry.addData("servo power:",power);
            telemetry.addData("Motor Speed:", motorSpeed);
            telemetry.addData("Target Speed:",expectedSpeed);
            telemetry.addData("Real Speed:",tmpmotor.getVelocity());
            telemetry.addData("spindexerPos:",spindexerpos);
            telemetry.addData("transferPos:",transferPos);
            telemetry.addLine("transferListPos:"+transferListPos+"("+ transferpositions[transferListPos%2]+")");
            telemetry.addLine("outtakePos:"+outtakePos+"("+outtakeslots[outtakePos%3]+")");
            telemetry.update();
        }
    }
}