package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;

@TeleOp

public class fullTest extends LinearOpMode{
    // drivetrain wheel motor declaration
    private DcMotor leftFront=null;
    private DcMotor leftBack=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack=null;
    //drivetrain configs
    private static final double DRIVE_SPEED = 0.7;
    private static final double STRAFE_SPEED = 0.5;
    private static final double TWIST_SPEED = 0.5;
    private static final double SECONDARY_DILATION = 0.25;
    //other motors
    private DcMotorEx flywheelR =null;
    private DcMotorEx flywheel = null;
    private DcMotorEx intake=null;
    private DcMotor transfer=null;
    //servos
    private CRServo hoodServo=null;
    private CRServo spindexer=null;
    private CRServo turret=null;
    AnalogInput hoodAnalog=null;
    outtakeV2 outtake=null;
    //sensitivity(& other configs)
    double flywheelSensitivity=10;
    double hoodspeed=0.5;
    VoltageSensor battery=null;
    //FTC dashboard telemetry
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;
    //vars
    int flywheelspeed=2000;
    int targetspeed=0;
    int distance=0;
    boolean flywheelToggle=false;
    boolean transferToggle=false;
    double spindexerpos=0.75;
    double spindexerDialation=0.01;
    double epsilon=0.01;
    double[] spindexerpositions = {0,0.75};
    int spindexerAutoPos=1;
    boolean pasty=false;
    //button state storage
    Gamepad previousgamepad1 = new Gamepad();

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront   = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");
        //Reverse motor directions where needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //other motors
        flywheelR= hardwareMap.get(DcMotorEx.class,"flywheelR");
        flywheel=hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake=(DcMotorEx) hardwareMap.get(DcMotor.class,"intake");
        //servos
        hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        transfer=hardwareMap.get(DcMotor.class,"par1");
        turret=hardwareMap.get(CRServo.class,"turretServo");
        hoodAnalog=hardwareMap.get(AnalogInput.class,"hoodAnalog");
        battery=hardwareMap.get(VoltageSensor.class,"Control Hub");
        outtake=new outtakeV2(hardwareMap,flywheel,flywheelR,"Red",leftFront,rightFront,leftBack,rightBack,hoodServo,hoodAnalog,transfer,false);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        //repeat until opmode ends
        while (opModeIsActive()){
            //flywheel
            if (gamepad2.dpad_up){
                flywheelspeed+=flywheelSensitivity;
            }
            else if (gamepad2.dpad_down && flywheelspeed-flywheelSensitivity>=0){
                flywheelspeed-=flywheelSensitivity;
            }
            else if (gamepad2.dpad_down && flywheelspeed-flywheelSensitivity<0){
                flywheelspeed=0;
            }
            //toggle
            if (gamepad2.yWasPressed()){
                flywheelToggle=!flywheelToggle;
                if (flywheelToggle) {
                    targetspeed=flywheelspeed;
                    flywheelR.setVelocity(targetspeed);
                    flywheel.setVelocity(targetspeed);
                } else{
                    targetspeed=0;
                    flywheelR.setVelocity(0);
                    flywheel.setVelocity(0);
                }
            }
            if (gamepad2.aWasPressed()) distance++;
            if (gamepad2.bWasPressed()) distance--;
            telemetry.addLine("Flywheel Speed:"+flywheelspeed+" encoder ticks/s, "+flywheelspeed*60/28+" RPM");
            telemetry.addLine("Flywheel Target Speed:"+targetspeed+" encoder ticks/s, "+targetspeed*60/28+" RPM");
            telemetry.addLine("Flywheel Real Speed:"+flywheelR.getVelocity()+" encoder ticks/s, "+flywheelR.getVelocity()*60/28+" RPM");
            telemetry.addData("Hood voltage",hoodAnalog.getVoltage());
            dashboardTelemetry.addData("Flywheel Target Speed ets",targetspeed);
            dashboardTelemetry.addData("Flywheel Target Speed rpm",targetspeed*60/28);
            dashboardTelemetry.addData("Flywheel Real Speed ets",-flywheelR.getVelocity());
            dashboardTelemetry.addData("Flywheel Real Speed rpm",-flywheelR.getVelocity()*60/28);
            dashboardTelemetry.addData("Hood voltage",hoodAnalog.getVoltage());
            dashboardTelemetry.addData("Battery Voltage",battery.getVoltage());
            dashboardTelemetry.addData("Distance",distance);
            //spindexer
//            if (spindexerpos-gamepad1.left_stick_x*spindexerDialation>=0 && spindexerpos-gamepad1.left_stick_x*spindexerDialation<=0.75){
//                spindexerpos-=gamepad1.left_stick_x*spindexerDialation;
//            }
//            else if (spindexerpos-gamepad1.left_stick_x*spindexerDialation<0){
//                spindexerpos=0;
//            }
//            else if (spindexerpos-gamepad1.left_stick_x*spindexerDialation>0.75){
//                spindexerpos=0.75;
//            }
            if (gamepad2.right_bumper){
                spindexer.setPower(1);
            }else if (gamepad2.left_bumper){
                spindexer.setPower(-1);
            }else{
                spindexer.setPower(0);
            }
//            if ((spindexer.getPosition()<=spindexerpositions[0]+epsilon) && (spindexer.getPosition()>=spindexerpositions[0]-epsilon) && spindexerAutoPos%2==0){
//                spindexerAutoPos++;
//                spindexerpos=spindexerpositions[spindexerAutoPos%2];
//            }
            //update gamepad+telemetry
            previousgamepad1.copy(gamepad1);
            //transfer
            if (gamepad2.xWasPressed()){
                transferToggle=!transferToggle;
            }
            if (transferToggle) {
                transfer.setPower(1);
                telemetry.addLine("Transfer Position: Up");
            }else{
                transfer.setPower(0);
                telemetry.addLine("Transfer Position: Stopped");
            }
//            telemetry.addData("transfer Target Speed:",transfer.getPosition());
            //hood
            if (gamepad2.dpad_right){
                hoodServo.setPower(hoodspeed);
            }
            else if (gamepad2.dpad_left){
                hoodServo.setPower(-hoodspeed);
            }
            else{
                hoodServo.setPower(0);
            }

            //intake
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            //turret
            turret.setPower(-gamepad2.right_stick_x * 0.5);

            //drivetrain (gamepad1)
            updateDrivetrain();
            telemetry.update();
            dashboardTelemetry.update();
        }
    }

    private void updateDrivetrain() {
        // Base control from analog sticks
        double drive = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = -gamepad1.left_stick_x * STRAFE_SPEED;
        double twist = -gamepad1.right_stick_x * TWIST_SPEED;

        // D-pad overrides for precise movement
        if (gamepad1.dpad_up) drive = DRIVE_SPEED;
        if (gamepad1.dpad_down) drive = -DRIVE_SPEED;
        if (gamepad1.dpad_left) strafe = STRAFE_SPEED;
        if (gamepad1.dpad_right) strafe = -STRAFE_SPEED;

        // Face buttons for slow precise movement
        if (gamepad1.y) drive = DRIVE_SPEED * SECONDARY_DILATION;
        if (gamepad1.a) drive = -DRIVE_SPEED * SECONDARY_DILATION;
        if (gamepad1.x) strafe = STRAFE_SPEED * SECONDARY_DILATION;
        if (gamepad1.b) strafe = -STRAFE_SPEED * SECONDARY_DILATION;

        double[] speeds = {
                (drive - strafe - twist), // leftFront
                (drive + strafe + twist), // rightFront
                (drive + strafe - twist), // leftBack
                (drive - strafe + twist)  // rightBack
        };

        double max = Math.abs(speeds[0]);
        for (int i = 1; i < speeds.length; i++) {
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
