package org.firstinspires.ftc.teamcode.Stanley.finalizedClasses;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Aaron.aprilTagV3;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class outtakeV3 {
    //Team color
    String teamColor;

    //================================  April Tag  ================================
    //April tag processor
    public aprilTagV3 apriltag;
    //Target Apriltag pipeline
    public int aprilTagPipeline=5;

    //================================  Flywheel  ================================
    //Outtake flywheel
    public DcMotorEx flywheelDriveR;
    public DcMotorEx flywheelDrive;

    //================================  Transfer  ================================
    //transfer servo
    public DcMotor transfer;
    //transfer positions(up, down)
    public static double[] transferpowers ={1,0};

    //================================  Hood  ================================
    //Motor for hood encoder
    public DcMotor hoodEncoder=null;
    //Outtake Hood Servo
    public CRServo hoodServo;
    public CRServo turretServo;
    //Degrees changed for every servo rotation
    public double servoDegPerRot =24.18;
    //Encoder tick offset for encoder
    public int encoderOffset=0;
    //Ticks/revolution for encoder
    public int ticksPerRevHood=8192;
    //PID instance for hood
    public double[] Kh={0.0005,0.0005,0.00003};
    public PID hoodPID=new PID(Kh[0],Kh[1],Kh[2]);
    
    //================================  Turret  ================================
    //Robot drivetrain object
    MecanumDrive drive = null;
    //Acceptable angle rotation range of turret(0=right, 90=up, 180=left, 270=down)
    public double minTurretAngle=0;
    public double maxTurretAngle=180;
    //Turret autoaim epsilon
    public double turretEpsilon=1.5;
    //auto aim vars
    //  Drive = Error * Gain
    // Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    // P, I, D
    public double[] Kturn={0.015,0.0002,0.04};
    public PID turnPID=new PID(Kturn[0],Kturn[1],Kturn[2]);

    //================================  Config  ================================
    //vars
    int targetTagID=-1;
    //voltage jump to be considered a rotation
    double maxVJump=3.3*0.5;
    //to use turret rotation limitation or not(requires initialized MecanumDrive
    public boolean setRange=false;

    /**
     * Constructor
     * @param hardwareMap Hardwaremap
     * @param teamColor Team color("Red" or "Blue")
     * @param useTag To use april tag process or not
     * @param drive Robot drivetrain
     */
    public outtakeV3(HardwareMap hardwareMap, String teamColor, boolean useTag, MecanumDrive drive){
        this.flywheelDriveR = hardwareMap.get(DcMotorEx.class,"flywheelR");
        this.flywheelDrive=hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheelDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        this.drive=drive;
        this.teamColor=teamColor;
        this.hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        this.turretServo =hardwareMap.get(CRServo.class,"turretServo");
        this.hoodEncoder=hardwareMap.get(DcMotorEx.class,"leftBack");
        this.transfer=hardwareMap.get(DcMotor.class,"par1");
        hoodPID.maxOutput=1;
        hoodPID.minOutput=-1;
        //set target april tag number to aim at depending on team color.
        if (Objects.equals(this.teamColor, "Red") && this.targetTagID!=-1){
            this.targetTagID=24;
        }
        else if (Objects.equals(this.teamColor, "Blue") && this.targetTagID!=-1) {
            this.targetTagID = 20;
        }
        //Init apriltag instance
        if (useTag) {
            this.apriltag = new aprilTagV3(hardwareMap);
            this.apriltag.setPipeline(5);
            this.apriltag.init();
        }
    }

    /**
     * Helper function to normalize angle to +-180 range
     * @param angle
     * @return
     */
    public static double normalizeAngle(double angle) {
        angle = angle % 360.0;
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    /**
     * Autoaim to april tag.
     * Target tag set by teamColor variable in class, "Red" or "Blue"
     * @return False if canceled or teamColor not found, True if successful
     */
    public boolean autoturn(){
        setPipeLine(aprilTagPipeline);


        this.apriltag.scanOnce();
        if (!apriltag.hasValidTarget()){
            turretServo.setPower(0);
            turnPID=new PID(Kturn[0],Kturn[1],Kturn[2]);
            if (setRange) {
                //get current heading
                double currentHeading = drive.localizer.getPose().heading.toDouble();
                //Shift heading from 180 to -180 to 0 to 360
                if (currentHeading < 0) {
                    currentHeading += 360;
                }
                //get current position
                Vector2d currentPos = drive.localizer.getPose().position;
                // Robot position
                double xr = currentPos.x;
                double yr = currentPos.y;

                // Target position
                double xt = -61.5;
                double yt = 0;
                if (Objects.equals(this.teamColor, "Red")) {
                    yt = 52.5;
                } else if (Objects.equals(this.teamColor, "Blue")) {
                    yt = -53.5;
                }

                // Robot facing angle in degrees (-180 to 180)
                double robotAngle = currentHeading;

                // Compute vector to target
                double dx = xt - xr;
                double dy = yt - yr;

                // Target angle in world coordinates (-180 to 180)
                double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

                // Angle of target relative to robot front
                double relativeAngle = normalizeAngle(targetAngle - robotAngle);

                // Attempt to move camera within range
                if (relativeAngle > maxTurretAngle) {
                    turretServo.setPower(0.1);
                } else if (relativeAngle < minTurretAngle) {
                    turretServo.setPower(-0.1);
                }
            }
            return false;
        }
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  headingError    = apriltag.getYaw();

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double power   = turnPID.update(headingError) ;
        turretServo.setPower(-power);
        return Math.abs(apriltag.getYaw())<=turretEpsilon;
    }

    /**
     * Get distance to april tag using limelight
     * @return Distance in in.
     */
    public double getDistance(){
        /*
          Formula:
          d=(h2-h1)/tan (a1+a2)
          h1: Height of the Limelight lens from the floor.
          h2: Height of the target (AprilTag) from the floor.
          a1: Mounting angle of the Limelight (degrees back from vertical).
          a2: Vertical offset angle to the target, obtained from the Limelight's ty value.
         */
        return (29.5-11.4375)/Math.tan(Math.toRadians(20+apriltag.getPitch()));
    }

    /**
     * Calculates exit speed for a curved hood system.
     * @param rpm Flywheel rotations per minute.
     * @param wheelDiameter Diameter of the flywheel (meters).
     * @param efficiency Realistic efficiency (0.90 to 0.98 for curved hoods).
     * @return Realistic exit speed in meters per second.
     */
    public double calculateCurvedExitSpeed(double rpm, double wheelDiameter, double efficiency) {
        // Tangential velocity: (RPM * PI * D) / 60
        double tangentialVelocity = (rpm * Math.PI * wheelDiameter) / 60.0;

        // Theoretical exit speed is exactly half of tangential velocity
        double theoreticalSpeed = tangentialVelocity / 2.0;

        // Apply hood efficiency to account for minor slip and compression losses
        return theoreticalSpeed * efficiency;
    }

    /**
     * Calculates the required RPM to reach a target ball exit velocity.
     *
     * @param targetVelocity The desired ball speed (e.g., meters per second).
     * @param diameter The diameter of the flywheel (same units as velocity, e.g., meters).
     * @param efficiency The slip/compression factor (0.0 to 1.0).
     *                   Use 0.90 - 0.95 for a well-tuned curved hood.
     * @return The required motor speed in RPM.
     */
    public double calculateRequiredRPM(double targetVelocity, double diameter, double efficiency) {
        // Validation to avoid division by zero
        if (diameter <= 0 || efficiency <= 0) return 0;

        // Requirement: Wheel surface speed = 2 * Target Ball Speed
        double requiredSurfaceVelocity = targetVelocity * 2.0;

        // Convert Surface Velocity to RPM:
        // RPM = (v_surface * 60) / (pi * diameter)
        double theoreticalRPM = (requiredSurfaceVelocity * 60.0) / (Math.PI * diameter);

        // Adjust for efficiency (lower efficiency requires higher RPM)
        return theoreticalRPM / efficiency;
    }

    public void setPipeLine(int pipeline){
        apriltag.setPipeline(pipeline);
    }
    /**
     * Find optimal launch angle and velocity to hit a target with specific impact angle constraints.
     * Theoretical launch velocity calculation:
     * PI*Diameter*RPM/60/2,
     * /2 only for single flywheel.
     * Note this value is assuming perfect conditions with 0 friction & slippage, actual values may be lower.
     *
     * @param targetX horizontal distance to target (inches)
     * @param targetY vertical height of target above launch point (inches)
     * @param minAngleDeg minimum hood angle range (degrees from horizontal)
     * @param maxAngleDeg maximum hood angle range (degrees from horizontal)
     * @param maxVelocity maximum launch velocity (inches/second)
     * @param minImpactAngleDeg minimum acceptable impact angle (degrees from horizontal)
     * @param maxImpactAngleDeg maximum acceptable impact angle (degrees from horizontal)
     * @param optimalImpactAngleDeg impact angle program should attempt to hit
     * @param g gravitational acceleration (inches/s^2, default 386.4 for Earth)
     * @param yEpsilon acceptable y-position error for hitting target (inches)
     * @param angleEpsilon convergence threshold for angle binary search (degrees)
     * @param maxIterations maximum iterations before giving up
     * @return Map<string,string> with keys: angle, velocity, impactAngle, yError, impactAngleError, success, reason
     */
    public Map<String,String> findOptimalLaunch(double targetX, double targetY,
                                                       double minAngleDeg, double maxAngleDeg,
                                                       double maxVelocity,
                                                       double minImpactAngleDeg, double maxImpactAngleDeg, double optimalImpactAngleDeg,
                                                       double g, double yEpsilon, double angleEpsilon,
                                                       int maxIterations) {

        // Convert angles to radians for calculations
        double minAngle = Math.toRadians(minAngleDeg);
        double maxAngle = Math.toRadians(maxAngleDeg);
        double minImpactAngle = Math.toRadians(minImpactAngleDeg);
        double maxImpactAngle = Math.toRadians(maxImpactAngleDeg);
        double medianImpactAngle = Math.toRadians(optimalImpactAngleDeg);

        // Track best solution found so far
        Map<String,String> bestSolution = null;
        double bestDistanceToMedian = Double.POSITIVE_INFINITY;

        // Binary search on launch angle
        double lowAngle = minAngle;
        double highAngle = maxAngle;

        String reason = "";

        for (int iteration = 0; iteration < maxIterations; iteration++) {
            // Check convergence: if angle range is very small, we've converged
            if ((highAngle - lowAngle) < Math.toRadians(angleEpsilon)) {
                reason = "converged";
                break;
            }

            double midAngle = (lowAngle + highAngle) / 2.0;

            // Binary search on velocity for this angle
            double foundVelocity = -1;
            double lowVelocity = 0;
            double highVelocity = maxVelocity;

            for (int velIteration = 0; velIteration < 100; velIteration++) {
                double midVelocity = (lowVelocity + highVelocity) / 2.0;

                // Calculate y position at target x using projectile motion equation
                // y = x*tan(θ) - (g*x²)/(2*v²*cos²(θ))
                double yAtTarget;
                try {
                    double cosAngle = Math.cos(midAngle);
                    if (midVelocity == 0 || cosAngle == 0) {
                        // Velocity too low, causes numerical issues
                        lowVelocity = midVelocity;
                        continue;
                    }
                    yAtTarget = targetX * Math.tan(midAngle) -
                            (g * targetX * targetX) / (2.0 * midVelocity * midVelocity * cosAngle * cosAngle);
                } catch (Exception e) {
                    // Velocity too low, causes numerical issues
                    lowVelocity = midVelocity;
                    continue;
                }

                // Check if we hit the target within epsilon
                if (Math.abs(yAtTarget - targetY) < yEpsilon) {
                    foundVelocity = midVelocity;
                    break;
                } else if (yAtTarget < targetY) {
                    // Undershooting, need more velocity
                    lowVelocity = midVelocity;
                } else {
                    // Overshooting, need less velocity
                    highVelocity = midVelocity;
                }
            }

            // Case 1: No valid velocity found (even max velocity undershoots)
            if (foundVelocity == -1) {
                // At max velocity, check if we're undershooting or can't reach
                double cosAngle = Math.cos(midAngle);
                double yAtTargetMaxVel = targetX * Math.tan(midAngle) -
                        (g * targetX * targetX) / (2.0 * maxVelocity * maxVelocity * cosAngle * cosAngle);

                if (yAtTargetMaxVel < targetY) {
                    // We're undershooting - could be angle too low OR too high
                    // Check if we can even reach target_x horizontally (max range formula)
                    double maxRange = (maxVelocity * maxVelocity * Math.sin(2.0 * midAngle)) / g;

                    if (maxRange < targetX) {
                        // Can't reach target horizontally - angle too high (too steep)
                        highAngle = midAngle;
                    } else {
                        // Can reach horizontally but undershooting vertically
                        // Need to check vertex position to determine if angle is too high or too low
                        double tVertex = maxVelocity * Math.sin(midAngle) / g;
                        double vertexX = maxVelocity * Math.cos(midAngle) * tVertex;

                        if (vertexX < targetX) {
                            // Vertex before target - on descending part of parabola
                            // Undershooting means angle is too high (falling too steeply)
                            highAngle = midAngle;
                        } else {
                            // Vertex at/after target - on ascending part of parabola
                            // Undershooting means angle is too low (not enough height)
                            lowAngle = midAngle;
                        }
                    }
                }
            }
            // Case 2: Valid velocity found, check impact angle
            else {
                // Calculate impact angle (angle of velocity vector at target)
                // v_x = v*cos(θ) (constant)
                // v_y = v*sin(θ) - g*t
                // At target x: t = x/(v*cos(θ))
                double tImpact = targetX / (foundVelocity * Math.cos(midAngle));
                double vX = foundVelocity * Math.cos(midAngle);
                double vY = foundVelocity * Math.sin(midAngle) - g * tImpact;

                // Impact angle from horizontal (negative v_y means going down)
                // atan2 gives angle from positive x-axis
                double impactAngle = Math.atan2(vY, vX);
                // Convert to 0-180 range
                if (impactAngle < 0) {
                    impactAngle = Math.PI + impactAngle;  // Convert negative angles
                }

                // Check if impact angle is within acceptable range
                if (minImpactAngle <= impactAngle && impactAngle <= maxImpactAngle) {
                    // Valid solution! Check if it's closer to median than previous best
                    double distanceToMedian = Math.abs(impactAngle - medianImpactAngle);
                    // Calculate y position at target x using projectile motion equation
                    // y = x*tan(θ) - (g*x²)/(2*v²*cos²(θ))
                    double yAtTarget;
                    double cosAngle = Math.cos(midAngle);
                    yAtTarget = targetX * Math.tan(midAngle) -
                            (g * targetX * targetX) / (2.0 * foundVelocity * foundVelocity * cosAngle * cosAngle);
                    if (distanceToMedian <= bestDistanceToMedian) {
                        bestDistanceToMedian = distanceToMedian;
                        bestSolution = new HashMap<>(Map.of(
                                "angle",Double.toString(Math.toDegrees(midAngle)),
                                "velocity",Double.toString(foundVelocity),
                                "impactAngle",Double.toString(Math.toDegrees(impactAngle)),
                                "yError",Double.toString(Math.abs(targetY-yAtTarget)), // Within epsilon by definition
                                "impactAngleError",Double.toString(distanceToMedian), // Within range by definition
                                "success","true",
                                "reason","valid_solution"
                        ));
                    }

                    // Update angle search based on impact angle relative to median
                    if (impactAngle < medianImpactAngle) {
                        // Impact too shallow, launch angle too low
                        highAngle = midAngle;
                    } else {
                        // Impact too steep, launch angle too high
                        lowAngle = midAngle;
                    }
                } else {
                    // Impact angle outside acceptable range
                    if (impactAngle<=Math.PI/2){
                        //On ascending part of parabola, launch angle too low
                        lowAngle=midAngle;
                    }
                    else if (impactAngle > maxImpactAngle) {
                        // Impact too steep, launch angle too high
                        highAngle = midAngle;
                    } else {
                        // Impact too shallow, launch angle too low
                        lowAngle = midAngle;
                    }
                    // Calculate y position at target x using projectile motion equation
                    // y = x*tan(θ) - (g*x²)/(2*v²*cos²(θ))
                    double yAtTarget;
                    double cosAngle = Math.cos(midAngle);
                    yAtTarget = targetX * Math.tan(midAngle) -
                            (g * targetX * targetX) / (2.0 * foundVelocity * foundVelocity * cosAngle * cosAngle);
                    // Store as backup solution if we don't find anything better
                    double impactAngleError = Math.min(Math.abs(impactAngle - minImpactAngle),
                            Math.abs(impactAngle - maxImpactAngle));
                    if (bestSolution == null || impactAngleError < Double.parseDouble(bestSolution.get("impactAngleError"))) {
                        bestSolution = new HashMap<>(Map.of(
                                "angle",Double.toString(Math.toDegrees(midAngle)),
                                "velocity",Double.toString(foundVelocity),
                                "impactAngle",Double.toString(Math.toDegrees(impactAngle)),
                                "yError",Double.toString(Math.abs(targetY-yAtTarget)), // Within epsilon by definition
                                "impactAngleError",Double.toString(impactAngleError), // Within range by definition
                                "success","false",
                                "reason","impact_angle_out_of_range"
                        ));
                    }
                }
            }
        }

        // If loop completed without convergence
        if (reason.isEmpty()) {
            reason = "max_iterations";
        }

        // If we found a valid solution, return it
        if (bestSolution != null && Boolean.parseBoolean(bestSolution.get("success"))) {
            return bestSolution;
        }

        // Otherwise, return best effort
        if (bestSolution == null) {
            // No solution attempted velocity at all
            bestSolution = new HashMap<>(Map.of(
                    "angle",Double.toString(-1.0),
                    "velocity",Double.toString(-1.0),
                    "impactAngle",Double.toString(-1.0),
                    "yError",Double.toString(Double.POSITIVE_INFINITY), // Within epsilon by definition
                    "impactAngleError",Double.toString(Double.POSITIVE_INFINITY), // Within range by definition
                    "success","false",
                    "reason","no_solution_found"
            ));
        }

        return bestSolution;
    }

    /**
     * Set the hood angle to a specific degree
     * @param degrees degrees from hood to horizontal(cnt. clockwise)
     * @return if hood is at position
     */
    public boolean setHood(double degrees){
        double epsilon=40;
        double targetTicks=(66.81-degrees)/servoDegPerRot*ticksPerRevHood;
        double power=hoodPID.update(targetTicks+hoodEncoder.getCurrentPosition()+encoderOffset);
        hoodServo.setPower(-power);
        return (hoodEncoder.getCurrentPosition()+encoderOffset >= targetTicks - epsilon) && (hoodEncoder.getCurrentPosition()+encoderOffset <= targetTicks + epsilon);
    }

    /**
     * Auto-initializes hood(spin off highest, then reset angle counter)
     * WARNING:Blocking
     */
    public void initHoodAngleBlocking(){
        hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        while((hoodEncoder.getCurrentPosition()+encoderOffset)> -3*ticksPerRevHood) hoodServo.setPower(1);
        hoodServo.setPower(0);
        hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Resets hood angle counter to highest(gear off)(AKA 0)
     * Will cause left-back drivetrain motor to temporarily lose power,
     * FOR USE IN EMERGENCIES ONLY
     */
    public void resetHoodAngle(){
        hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoderOffset=0;
    }

    /**
     * Stops hood servo
     */
    public void stopHood(){
        hoodServo.setPower(0);
    }

    /**
     * Transfer artifact to flywheel(move transfer up)
     */
    public void transferUp(){this.transfer.setPower(transferpowers[0]);}
    /**
     * Lower Transfer
     */
    public void transferDown(){
        this.transfer.setPower(transferpowers[1]);
    }

    /**
     * Spin flywheel to speed
     * @param targetSpeed Target flywheel speed in encoder ticks/sec.
     * @param tolerance Tolerance in ticks/sec. from target speed to return true.
     * @return If flywheel is up to speed.
     */
    public boolean spin_flywheel(double targetSpeed, int tolerance){
        DcMotorEx flywheelDriveEx=this.flywheelDriveR;
        flywheelDriveEx.setVelocity(targetSpeed);
        flywheelDrive.setVelocity(targetSpeed);
        return targetSpeed - tolerance <= flywheelDriveEx.getVelocity() && flywheelDriveEx.getVelocity() <= targetSpeed + tolerance;
    }
}
