package org.firstinspires.ftc.teamcode.Alvin;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="intakeTest (Alvin)", group="Test")
public class intakeTest extends LinearOpMode {

    // TODO: change these to match your Robot Configuration names
    private static final String INTAKE_MOTOR_NAME = "intake";
    private static final String COLOR_SENSOR_NAME = "intakeSensor";

    // Intake-until-pixel test settings
    private static final long UNTIL_PIXEL_TIMEOUT_MS = 2500;

    // Edge detection so holding a button doesn't retrigger repeatedly
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevLB = false;
    private boolean prevRB = false;

    @Override
    public void runOpMode() {
        intake in = new intake(hardwareMap, INTAKE_MOTOR_NAME, COLOR_SENSOR_NAME);

        boolean runningUntilPixel = false;
        boolean lastDetected = false;
        long lastStartMs = 0;

        telemetry.addLine("IntakeTest ready.");
        telemetry.addLine("A: intake (forward)   B: reverse   X: stop");
        telemetry.addLine("Y: start/stop intakeUntilPixel()");
        telemetry.addLine("LB: color light OFF   RB: color light ON");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y){
                in.intakeUntilPixel();
            }else{
                in.intakeMotor.setPower(0);
            }

            telemetry.addData("Detected",in.isPixelDetected());
            telemetry.update();
        }
    }
}
