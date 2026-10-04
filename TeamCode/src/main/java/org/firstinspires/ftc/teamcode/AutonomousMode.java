package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * Autonomous program for FTC BIO Buzz.
 * Features:
 * 1. GoBILDA Pinpoint + 4-Bar Odometry localization
 * 2. AprilTag camera initialization scanning
 * 3. Drive & Strafe positioning using Pinpoint pose feedback
 * 4. Auto-Aiming turret using field localization and turret lateral offset
 * 5. Flywheel shooter spin up, blocker servo opening, and separate intake & transfer feed
 * 6. Front wedge flower mechanism deployment
 */
@Autonomous(name = "BIO Buzz Autonomous", group = "Autonomous")
public class AutonomousMode extends LinearOpMode {

    private final HardwareRobot robot = new HardwareRobot();

    // Field Target Goal Coordinates (Inches)
    private static final double TARGET_GOAL_X = 72.0;
    private static final double TARGET_GOAL_Y = 72.0;

    // Turret Lateral Offset relative to robot center (Inches)
    private static final double TURRET_OFFSET_X = -2.0;
    private static final double TURRET_OFFSET_Y = 0.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Hardware & GoBILDA Pinpoint...");
        telemetry.update();

        robot.init(hardwareMap);
        robot.initVision();

        int detectedTagId = -1;

        // Scan during initialization phase
        while (!isStarted() && !isStopRequested()) {
            Pose2D pose = robot.updatePinpoint();
            telemetry.addData("Pinpoint Pose", "X:%.1f Y:%.1f Heading:%.1f",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));

            List<AprilTagDetection> detections = robot.getAprilTagDetections();
            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    if (detection instanceof AprilTagSingleDetection) {
                        AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                        detectedTagId = singleDet.id;
                        telemetry.addData("Init Scan", "Detected AprilTag ID: %d", detectedTagId);
                        break;
                    }
                }
            } else {
                telemetry.addData("Init Scan", "Scanning for AprilTags...");
            }
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Autonomous Running...");
        telemetry.update();

        // ----------------------------------------------------
        // AUTONOMOUS MATCH SEQUENCE
        // ----------------------------------------------------

        // 1. Drive forward toward shooting position
        driveToPosePinpoint(0.0, 24.0, 0.0, 3.0);

        // 2. Auto-Aim Turret using GoBILDA Pinpoint Localization
        double targetTurretAngle = robot.calculateTurretAutoAimAngle(
                TARGET_GOAL_X, TARGET_GOAL_Y, TURRET_OFFSET_X, TURRET_OFFSET_Y);

        if (robot.turretMotor != null) {
            double kP = 0.03;
            double turretPower = Math.min(Math.max(targetTurretAngle * kP, -0.6), 0.6);
            robot.turretMotor.setPower(turretPower);
            sleep(500);
            robot.turretMotor.setPower(0.0);
        }

        // 3. Spin up Flywheel Shooter & Open Blocker Servo
        if (robot.shooterMotor != null) {
            robot.shooterMotor.setPower(0.85);
        }
        robot.openBlocker();
        sleep(1000); // Allow flywheel time to reach full speed

        // 4. Feed balls/elements with Transfer & Intake Motors
        if (robot.intakeMotor != null)   robot.intakeMotor.setPower(0.7);
        if (robot.transferMotor != null) robot.transferMotor.setPower(0.9);
        sleep(2500); // Feed elements into flywheel

        // Stop Intake, Transfer, Shooter, and Close Blocker
        if (robot.intakeMotor != null)   robot.intakeMotor.setPower(0.0);
        if (robot.transferMotor != null) robot.transferMotor.setPower(0.0);
        if (robot.shooterMotor != null)  robot.shooterMotor.setPower(0.0);
        robot.closeBlocker();

        // 5. Deploy Flower Front Wedge Mechanism
        robot.deployFlower();

        // 6. Strafe right to designated parking position
        driveToPosePinpoint(18.0, 24.0, 0.0, 3.0);

        telemetry.addData("Status", "Autonomous Sequence Complete!");
        telemetry.update();

        robot.closeVision();
    }

    /**
     * Drive to target field position (targetX, targetY) using GoBILDA Pinpoint odometry feedback.
     */
    public void driveToPosePinpoint(double targetX, double targetY, double targetHeadingDeg, double timeoutS) {
        if (!opModeIsActive()) return;

        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < timeoutS * 1000)) {
            Pose2D pose = robot.updatePinpoint();
            double currentX = pose.getX(DistanceUnit.INCH);
            double currentY = pose.getY(DistanceUnit.INCH);
            double currentHeading = pose.getHeading(AngleUnit.DEGREES);

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double errorHeading = targetHeadingDeg - currentHeading;

            if (Math.hypot(errorX, errorY) < 1.0 && Math.abs(errorHeading) < 2.0) {
                break;
            }

            double kP_drive = 0.05;
            double kP_turn  = 0.03;

            double fieldYPower = errorY * kP_drive;
            double fieldXPower = errorX * kP_drive;
            double turnPower   = errorHeading * kP_turn;

            double headingRad = Math.toRadians(currentHeading);
            double robotYPower = fieldYPower * Math.cos(headingRad) + fieldXPower * Math.sin(headingRad);
            double robotXPower = -fieldYPower * Math.sin(headingRad) + fieldXPower * Math.cos(headingRad);

            double denominator = Math.max(Math.abs(robotYPower) + Math.abs(robotXPower) + Math.abs(turnPower), 1.0);
            double fl = (robotYPower + robotXPower + turnPower) / denominator;
            double bl = (robotYPower - robotXPower + turnPower) / denominator;
            double fr = (robotYPower - robotXPower - turnPower) / denominator;
            double br = (robotYPower + robotXPower - turnPower) / denominator;

            if (robot.leftFront != null)  robot.leftFront.setPower(fl * 0.7);
            if (robot.leftRear != null)   robot.leftRear.setPower(bl * 0.7);
            if (robot.rightFront != null) robot.rightFront.setPower(fr * 0.7);
            if (robot.rightRear != null)  robot.rightRear.setPower(br * 0.7);

            telemetry.addData("Driving to Target", "X:%.1f Y:%.1f Err:%.1f", targetX, targetY, Math.hypot(errorX, errorY));
            telemetry.update();

            sleep(20);
        }

        robot.stopDrivetrain();
    }
}
