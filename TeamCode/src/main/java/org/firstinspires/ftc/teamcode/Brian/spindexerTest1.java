package org.firstinspires.ftc.teamcode.Brian;
import org.firstinspires.ftc.teamcode.Aaron.aprilTag;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
@TeleOp
public class spindexerTest1 extends LinearOpMode{
    private DcMotor leftFront=null;
    private DcMotor leftBack=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack=null;
    private DcMotorEx flywheel=null;
    private Servo spindexerServo=null;
    private Limelight3A limelight;
    private aprilTag aprilTag;
    private spindexer spindexer;

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");
        spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize AprilTag system
        aprilTag = new aprilTag(hardwareMap);

        // Initialize spindexer
        //TODO: fix spindexer intialization, requires Servo spindexerServo
        spindexer = new spindexer(spindexerServo);
//        spindexer.spindexerServo = spindexerServo;

        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Scan for AprilTags
            aprilTag.scanOnce();

            // Get motif code from AprilTag (returns 1, 2, or 3, or -1 if not detected)
            int motifCode = aprilTag.getMotifCode();

            // Update spindexer motif pattern based on detected tag
            // motifCode: 1 = GPP (pattern index 0), 2 = PGP (pattern index 1), 3 = PPG (pattern index 2)
            if (motifCode >= 1 && motifCode <= 3) {
                int patternIndex = motifCode - 1; // Convert 1,2,3 to 0,1,2
                // Only update if pattern changed (to avoid resetting motifIndex unnecessarily)
                if (spindexer.currentMotifPattern != patternIndex) {
                    spindexer.currentMotifPattern = patternIndex;
                    spindexer.motifPattern = spindexer.motifPatterns[patternIndex];
                    spindexer.resetMotifIndex(); // Reset to start of new pattern
                }
            }

            //drivetrain
            double drive  = gamepad1.right_stick_y*0.7;
            final double strafe_speed=0.5;
            //Default:0.5
            double strafe = -gamepad1.right_stick_x*0.5;
            if (gamepad1.dpad_left){
                strafe=strafe_speed;
            }
            else if (gamepad1.dpad_right){
                strafe=-strafe_speed;
            }
            double twist  = -gamepad1.left_stick_x*0.5;

            // Telemetry
            telemetry.addData("drive: ", drive);
            telemetry.addData("strafe: ", strafe);
            telemetry.addData("twist: ", twist);
            telemetry.addData("Motif Code: ", motifCode);
            telemetry.addData("Motif Pattern: ", spindexer.currentMotifPattern >= 0 ?
                    spindexer.getMotifName(spindexer.currentMotifPattern) : "Not Detected");
            telemetry.addData("Tag ID: ", aprilTag.getTagId());
            telemetry.addData("Distance: ", aprilTag.getDistance());
            telemetry.update();

            double[] speeds = {
                    (drive - strafe + twist), //forward-left motor
                    (drive + strafe + twist), //forward-right motor
                    (-drive - strafe + twist), //back-left motor
                    (-drive + strafe + twist)  //back-right motor
            };

            // Loop through all values in the speeds[] array and find the greatest
            // *magnitude*.  Not the greatest velocity.
            double max = Math.abs(speeds[0]);
            for(int i = 0; i < speeds.length; i++) {
                if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
            }

            // If and only if the maximum is outside of the range we want it to be,
            // normalize all the other speeds based on the given speed value.
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            // apply the calculated values to the motors.
            leftFront.setPower(speeds[0]);
            rightBack.setPower(speeds[1]);
            leftBack.setPower(speeds[2]);
            rightFront.setPower(speeds[3]);
        }
    }
}