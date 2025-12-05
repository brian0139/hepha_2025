package org.firstinspires.ftc.teamcode.Brian;
import org.firstinspires.ftc.teamcode.Aaron.aprilTag;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
@TeleOp
public class spindexerTestWoCamera extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotorEx flywheel = null;
    private Servo spindexerServo = null;
    private Limelight3A limelight;
    private aprilTag aprilTag;
    private spindexer spindexer;

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize spindexer
        //TODO: fix spindexer intialization, requires Servo spindexerServo
        spindexer = new spindexer();
        spindexer.spindexerServo = spindexerServo;

        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();

        // Main loop
        while (opModeIsActive()) {

            boolean pastRbumper = false;
            boolean pastLbumper = false;

            while (opModeIsActive()) {
                //outtake test
                if (gamepad1.right_bumper && !pastRbumper) {
                    pastRbumper = true;
                    if (spindexer.currentPosition < 1) {
                        spindexer.currentPosition++;
                        spindexer.rotateSpindexerInput(spindexer.currentPosition);
                    } else {
                        pastRbumper = true;
                    }
                } else if (!gamepad1.right_bumper && pastRbumper) {
                    pastRbumper = false;
                }
                if (gamepad1.left_bumper && !pastLbumper) {
                    pastLbumper = true;
                    if (spindexer.currentPosition >= 0) {
                        spindexer.currentPosition++;
                        spindexer.rotateSpindexerInput(spindexer.currentPosition);
                    }
                } else if (!gamepad1.left_bumper && pastLbumper) {
                    pastLbumper = false;
                }

                //drivetrain
                double drive = gamepad1.right_stick_y * 0.7;
                final double strafe_speed = 0.5;
                //Default:0.5
                double strafe = -gamepad1.right_stick_x * 0.5;
                if (gamepad1.dpad_left) {
                    strafe = strafe_speed;
                } else if (gamepad1.dpad_right) {
                    strafe = -strafe_speed;
                }
                double twist = -gamepad1.left_stick_x * 0.5;

                // Telemetry
                telemetry.addData("drive: ", drive);
                telemetry.addData("strafe: ", strafe);
                telemetry.addData("twist: ", twist);
                telemetry.addData("Motif Pattern: ", spindexer.currentMotifPattern >= 0 ?
                        spindexer.getMotifName(spindexer.currentMotifPattern) : "Not Detected");

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
                for (int i = 0; i < speeds.length; i++) {
                    if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
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
}
