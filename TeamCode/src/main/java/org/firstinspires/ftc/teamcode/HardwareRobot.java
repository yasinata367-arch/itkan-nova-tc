package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Robot Hardware definition class for FTC BIO Buzz robot.
 * Drivetrain: Mecanum Drive (4 Motors)
 * Mechanisms:
 *   - Turret (with lateral offset)
 *   - Shooter Flywheel
 *   - Front Intake Motor
 *   - Back Transfer Motor (supports opposite direction spinning to prevent power draw/stall)
 *   - Blocker Servo (opens shooter pathway)
 *   - Flower Servo (pushes down front wedge mechanism)
 * Localization: GoBILDA Pinpoint Odometry Computer + 4-Bar Odometry Pods
 * Vision: Webcam 1 with AprilTag VisionPortal
 */
public class HardwareRobot {

    // Drivetrain Motors
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;

    // Mechanism Motors
    public DcMotor turretMotor = null;
    public DcMotor shooterMotor = null;
    public DcMotor intakeMotor = null;   // Front Intake Rollers
    public DcMotor transferMotor = null; // Back Transfer Rollers

    // Servos
    public Servo blockerServo = null; // Shooter gate / blocker servo
    public Servo flowerServo = null;  // Front wedge flower mechanism servo

    // Servo Constants
    public static final double BLOCKER_CLOSED_POS = 0.0;
    public static final double BLOCKER_OPEN_POS   = 0.5;

    public static final double FLOWER_RETRACTED_POS = 0.0;
    public static final double FLOWER_DEPLOYED_POS  = 0.8;

    // Localization & Sensors
    public GoBildaPinpointDriver pinpoint = null;
    public IMU imu = null;

    // Vision
    public AprilTagProcessor aprilTagProcessor = null;
    public VisionPortal visionPortal = null;

    // Hardware Map reference
    private HardwareMap hwMap = null;

    public HardwareRobot() {}

    /**
     * Initialize all hardware components from Driver Station configuration map.
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // 1. Drivetrain Motors Initialization
        leftFront  = getMotorSafe("leftFront");
        rightFront = getMotorSafe("rightFront");
        leftRear   = getMotorSafe("leftRear");
        rightRear  = getMotorSafe("rightRear");

        if (leftFront != null)  leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftRear != null)   leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightRear != null)  rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        setBrakeMode(true);
        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 2. Mechanism Motors Initialization
        turretMotor = getMotorSafe("turretMotor");
        if (turretMotor != null) {
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        shooterMotor = getMotorSafe("shooterMotor");
        if (shooterMotor != null) {
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        intakeMotor = getMotorSafe("intakeMotor");
        if (intakeMotor != null) {
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        transferMotor = getMotorSafe("transferMotor");
        if (transferMotor != null) {
            transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // 3. Servos Initialization
        blockerServo = getServoSafe("blockerServo");
        if (blockerServo != null) {
            blockerServo.setPosition(BLOCKER_CLOSED_POS);
        }

        flowerServo = getServoSafe("flowerServo");
        if (flowerServo != null) {
            flowerServo.setPosition(FLOWER_RETRACTED_POS);
        }

        // 4. GoBILDA Pinpoint Odometry Computer Initialization
        initPinpoint();

        // 5. Fallback IMU Initialization
        try {
            imu = hwMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            );
            imu.initialize(parameters);
        } catch (Exception ignored) {}

        stopDrivetrain();
    }

    public void openBlocker() {
        if (blockerServo != null) blockerServo.setPosition(BLOCKER_OPEN_POS);
    }

    public void closeBlocker() {
        if (blockerServo != null) blockerServo.setPosition(BLOCKER_CLOSED_POS);
    }

    public void deployFlower() {
        if (flowerServo != null) flowerServo.setPosition(FLOWER_DEPLOYED_POS);
    }

    public void retractFlower() {
        if (flowerServo != null) flowerServo.setPosition(FLOWER_RETRACTED_POS);
    }

    /**
     * Configure GoBILDA Pinpoint Odometry Computer with 4-Bar Odometry Pods.
     */
    public void initPinpoint() {
        try {
            pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");

            // Set odometry pod offsets in MM
            pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);

            // Configure 4-Bar Odometry Pods
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            // Encoder direction configuration
            pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            // Reset position to (0,0,0) and calibrate internal IMU
            pinpoint.resetPosAndIMU();
        } catch (Exception ignored) {}
    }

    /**
     * Update GoBILDA Pinpoint data and return current robot field pose.
     */
    public Pose2D updatePinpoint() {
        if (pinpoint != null) {
            pinpoint.update();
            return pinpoint.getPosition();
        }
        return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }

    /**
     * Calculate desired turret relative angle (degrees) to aim at field target (targetX, targetY).
     */
    public double calculateTurretAutoAimAngle(double targetX, double targetY, double turretOffsetX, double turretOffsetY) {
        Pose2D currentPose = updatePinpoint();
        double robotX = currentPose.getX(DistanceUnit.INCH);
        double robotY = currentPose.getY(DistanceUnit.INCH);
        double robotHeadingRad = currentPose.getHeading(AngleUnit.RADIANS);
        double robotHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);

        double turretFieldX = robotX + (turretOffsetX * Math.cos(robotHeadingRad) - turretOffsetY * Math.sin(robotHeadingRad));
        double turretFieldY = robotY + (turretOffsetX * Math.sin(robotHeadingRad) + turretOffsetY * Math.cos(robotHeadingRad));

        double deltaX = targetX - turretFieldX;
        double deltaY = targetY - turretFieldY;
        double targetFieldAngleDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));

        double turretTargetDeg = targetFieldAngleDeg - robotHeadingDeg;

        while (turretTargetDeg > 180)  turretTargetDeg -= 360;
        while (turretTargetDeg < -180) turretTargetDeg += 360;

        return turretTargetDeg;
    }

    /**
     * Initialize AprilTag Processor and VisionPortal.
     */
    public void initVision() {
        try {
            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hwMap.get(WebcamName.class, "Webcam 1"),
                aprilTagProcessor
            );
        } catch (Exception ignored) {}
    }

    private DcMotor getMotorSafe(String name) {
        try {
            return hwMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    private Servo getServoSafe(String name) {
        try {
            return hwMap.get(Servo.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    public void setBrakeMode(boolean brake) {
        DcMotor.ZeroPowerBehavior mode = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        if (leftFront != null)  leftFront.setZeroPowerBehavior(mode);
        if (rightFront != null) rightFront.setZeroPowerBehavior(mode);
        if (leftRear != null)   leftRear.setZeroPowerBehavior(mode);
        if (rightRear != null)  rightRear.setZeroPowerBehavior(mode);
    }

    public void stopDrivetrain() {
        if (leftFront != null)  leftFront.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
        if (leftRear != null)   leftRear.setPower(0);
        if (rightRear != null)  rightRear.setPower(0);
    }

    public void setDriveRunMode(DcMotor.RunMode mode) {
        if (leftFront != null)  leftFront.setMode(mode);
        if (rightFront != null) rightFront.setMode(mode);
        if (leftRear != null)   leftRear.setMode(mode);
        if (rightRear != null)  rightRear.setMode(mode);
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        if (aprilTagProcessor != null) {
            return aprilTagProcessor.getDetections();
        }
        return null;
    }

    public void closeVision() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
