package org.firstinspires.ftc.teamcode.Alvin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class intakeTest extends LinearOpMode {

    // Declare motor and sensor variables
    private DcMotor intake;
    private colorSensor intakeSensor;

    // Power values for the motor
    private double intakePower = 1.0;    // Forward power
    private double reversePower = -1.0;  // Reverse power
    private double stopPower = 0.0;      // Stop power

    // For managing the intake until a pixel is detected
    private boolean runningPixelIntake = false;

    @Override
    public void runOpMode() {

        // Initialize hardware
//        intake = hardwareMap.get(DcMotor.class, "intake");  // Assuming the motor name is "intake"
//        intakeSensor = new colorSensor(hardwareMap, "intakeSensor");  // Assuming the sensor name is "intakeSensor"
//        intakeSensor.enableLight(true);  // Enable the color light on the sensor
        intake intakeOperator=new intake(hardwareMap,"intake","intakeSensor");


        // Provide telemetry to indicate the OpMode is ready
        telemetry.addLine("IntakeTest OpMode Initialized");
        telemetry.addLine("A: Intake Forward | B: Reverse Intake | X: Stop Intake | Y: Start/Stop Intake Until Pixel Detected");
        telemetry.update();

        waitForStart();  // Wait for the start button to be pressed

        // Main loop
        while (opModeIsActive()) {
            boolean tmp=intakeOperator.intakeUntilPixel();
            telemetry.addData("Detected",intakeOperator.colorDetector.getDetected());
            telemetry.addData("tmp",tmp);
            telemetry.update();
        }
    }
}