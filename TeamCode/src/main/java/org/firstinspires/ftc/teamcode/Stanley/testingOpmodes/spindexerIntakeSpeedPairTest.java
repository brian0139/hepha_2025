package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class spindexerIntakeSpeedPairTest extends LinearOpMode {
    DcMotor intake;
    CRServo spindexer;
    //FTC dashboard telemetry
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;
    // Drivetrain motors
    DcMotor leftFront = null;
    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor rightBack = null;
    static final double DRIVE_SPEED = 0.7;
    static final double STRAFE_SPEED = 1;
    static final double TWIST_SPEED = 0.5;

    boolean correctingtoggle=false;
    double change=0.1;
    int x=0;
    //0=intake,1=spindexer
    double[] values={0.1,0.1};
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drivetrain motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Reverse motor directions where needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intake=hardwareMap.get(DcMotor.class,"intake");
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()){
            updateDrivetrain();
            if (gamepad1.yWasPressed()) correctingtoggle=!correctingtoggle;
            //shift speed
            if (gamepad1.rightBumperWasPressed()){
                change*=10;
            }else if (gamepad1.leftBumperWasPressed()){
                change/=10;
            }
            telemetry.addData("Change",change);
            //selection
            if (gamepad1.dpadLeftWasPressed()){
                x--;
                if (x<0){
                    x=1;
                }
            }
            if (gamepad1.dpadRightWasPressed()){
                x++;
                if (x>1){
                    x=0;
                }
            }
            if (gamepad1.dpadUpWasPressed()){
                values[x]+=change;
            }
            if (gamepad1.dpadDownWasPressed()){
                values[x]-=change;
            }
            String line1="Values: ";
            for (int i=0;i<=1;i++){
                if (i==x){
                    line1+="{";
                }
                values[i]=(double) Math.round(values[i] * Math.pow(10, 5)) / Math.pow(10, 5);
                line1+=values[i];
                if (i==x){
                    line1+="}";
                }
                line1+=", ";
            }
            if (correctingtoggle){
                spindexer.setPower(values[1]);
                intake.setPower(values[0]);
            }else{
                spindexer.setPower(0);
                intake.setPower(0);
            }
            telemetry.addLine(line1);
            telemetry.update();
        }
    }
    void updateDrivetrain() {
        // Base control from analog sticks
        double drive = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = -gamepad1.left_stick_x * STRAFE_SPEED;
        double twist = -gamepad1.right_stick_x * TWIST_SPEED;

        double[] speeds = {
                (drive - strafe - twist), // leftFront
                (drive + strafe + twist), // rightFront
                (drive + strafe - twist), // leftBack
                (drive - strafe + twist)  // rightBack
        };

        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (Math.abs(speeds[i]) > max) {
                max = Math.abs(speeds[i]);
            }
        }

        if (max > 1.0) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] /= max;
            }
        }

        leftFront.setPower(speeds[0]);
        rightFront.setPower(speeds[1]);
        leftBack.setPower(speeds[2]);
        rightBack.setPower(speeds[3]);
    }
}
