package org.firstinspires.ftc.teamcode.Shannon;
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

public class driveTrain extends LinearOpMode{
    // drivetrain wheel motor declaration
    private DcMotor wheel_0=null;
    private DcMotor wheel_1=null;
    private DcMotor wheel_2=null;
    private DcMotor wheel_3=null;

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        wheel_0   = hardwareMap.get(DcMotor.class, "wheel_0");
        wheel_1    = hardwareMap.get(DcMotor.class, "wheel_1");
        wheel_2   = hardwareMap.get(DcMotor.class, "wheel_2");
        wheel_3   = hardwareMap.get(DcMotor.class, "wheel_3");

        wheel_2.setDirection(DcMotor.Direction.REVERSE);
        wheel_3.setDirection(DcMotor.Direction.REVERSE);
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        //repeat untill opmode ends
        while (opModeIsActive()){
            //below is drivetrain
            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).
            double drive  = gamepad1.left_stick_y*0.7;
            final double strafe_speed=0.5;
            double strafe = -gamepad1.left_stick_x*0.5;
            if (gamepad1.dpad_left){
                strafe=strafe_speed;
            }
            else if (gamepad1.dpad_right){
                strafe=-strafe_speed;
            }
            double twist  = -gamepad1.right_stick_x;
            telemetry.addData("drive: ", drive);
            telemetry.addData("strafe: ", strafe);
            telemetry.addData("twist: ", twist);
            telemetry.update();

            double[] speeds = {
                    (drive + strafe + twist), //forward-left motor(wheel_0)
                    (drive - strafe - twist), //forward-right motor(wheel_1)
                    (drive - strafe + twist), //back-left motor(wheel_2)
                    (drive + strafe - twist)  //back-right motor(wheel_3)
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
            wheel_0.setPower(speeds[0]);
            wheel_3.setPower(speeds[1]);
            wheel_1.setPower(speeds[2]);
            wheel_2.setPower(speeds[3]);
        }
    }
}