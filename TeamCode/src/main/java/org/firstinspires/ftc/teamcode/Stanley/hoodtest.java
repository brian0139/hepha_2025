package org.firstinspires.ftc.teamcode.Stanley;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    boolean lasty=false;
    boolean lastx=false;
    double spindexerpos=0;
    double spindexerDialation=0.0005;
    double transferDialation=0.01;
    double[] outtakeslots = {60.0/360, 180.0/360, 300.0/360};
    double[] transferpositions ={0.6,0.9};
    double transferPos=transferpositions[1];
    int outtakePos=0;
    int transferListPos=0;
    boolean lastb=false;
    boolean lasta=false;
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
            if (gamepad1.y && !lasty){
                lasty=true;
                tmpmotor.setVelocity(motorSpeed);
                expectedSpeed = motorSpeed;
            }else if(!gamepad1.y && lasty){
                lasty=false;
            }
            if (gamepad1.x && !lastx) {
                lastx = true;
                tmpmotor.setVelocity(0);
                expectedSpeed = 0;
            }else if(!gamepad1.x && lastx){
                lastx=false;
            }
            tmpservo.setPower(power);
            if (spindexerpos-gamepad1.left_stick_x*spindexerDialation>=0 && spindexerpos-gamepad1.left_stick_x*spindexerDialation<=1){
                spindexerpos-=gamepad1.left_stick_x*spindexerDialation;
            }
            if (transferPos+gamepad1.left_stick_y*transferDialation>=0 && transferPos+gamepad1.left_stick_y*transferDialation<=1){
                transferPos+=gamepad1.left_stick_y*transferDialation;
            }
            if (gamepad1.a && !lasta){
                lasta=true;
                outtakePos++;
                spindexerpos=outtakeslots[outtakePos%3];
            }else if (!gamepad1.a){
                lasta=false;
            }
            if (gamepad1.b && !lastb){
                lastb=true;
                transferListPos++;
                transferPos= transferpositions[transferListPos%2];
            }else if (!gamepad1.b){
                lastb=false;
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