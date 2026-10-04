package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * TeleOp program for FTC BIO Buzz.
 * Controls:
 * Gamepad 1: Drivetrain (Mecanum translation/strafe/rotation, Right Bumper slow mode, 'A' to reset Pinpoint pose)
 * Gamepad 2:
 *   - Turret: Manual (Stick / D-Pad Left/Right) or Auto-Aim toggle ('B' button using GoBILDA Pinpoint localization)
 *   - Shooter: Flywheel variable (Right Trigger) or Preset ('A' button)
 *   - Blocker Servo: Opens when shooting or holding Right Bumper
 *   - Flower Servo (Front Wedge): D-Pad Up (Deploy) / D-Pad Down (Retract)
 *   - Intake & Transfer:
 *     - Left Trigger: Intake & Transfer Forward
 *     - Left Bumper: Reverse Both
 *     - 'X' Button: Spin-in-Place Mode (Intake Forward, Transfer Reverse to prevent power draw & stalls)
 *     - 'Y' Button: Transfer Feed Forward only
 */
@TeleOp(name = "BIO Buzz TeleOp", group = "TeleOp")
public class TeleOpMode extends LinearOpMode {

    private final HardwareRobot robot = new HardwareRobot();

    // Field Target Goal Coordinates (Inches)
    private static final double TARGET_GOAL_X = 72.0;
    private static final double TARGET_GOAL_Y = 72.0;

    // Turret Offset relative to robot center (Inches)
    private static final double TURRET_OFFSET_X = -2.0;
    private static final double TURRET_OFFSET_Y = 0.0;

    private boolean autoAimEnabled = false;
    private boolean lastBButton = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Hardware & GoBILDA Pinpoint...");
        telemetry.update();

        // Initialize robot hardware, Pinpoint odometry & vision
        robot.init(hardwareMap);
        robot.initVision();

        telemetry.addData("Status", "Initialized. Ready for Start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Pinpoint Odometry Pose
            Pose2D pose = robot.updatePinpoint();

            // Reset Pinpoint pose to (0,0,0) if Gamepad 1 'A' is pressed
            if (gamepad1.a) {
                if (robot.pinpoint != null) {
                    robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                }
            }

            // ----------------------------------------------------
            // 1. MECANUM DRIVETRAIN (Gamepad 1)
            // ----------------------------------------------------
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * 1.1;
            double rx =  gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            double speedMultiplier = gamepad1.right_bumper ? 0.5 : 1.0;

            if (robot.leftFront != null)  robot.leftFront.setPower(frontLeftPower * speedMultiplier);
            if (robot.leftRear != null)   robot.leftRear.setPower(backLeftPower * speedMultiplier);
            if (robot.rightFront != null) robot.rightFront.setPower(frontRightPower * speedMultiplier);
            if (robot.rightRear != null)  robot.rightRear.setPower(backRightPower * speedMultiplier);

            // ----------------------------------------------------
            // 2. TURRET & AUTO-AIM (Gamepad 2)
            // ----------------------------------------------------

            // Toggle Auto-Aim mode with 'B' button press
            if (gamepad2.b && !lastBButton) {
                autoAimEnabled = !autoAimEnabled;
            }
            lastBButton = gamepad2.b;

            double turretPower;

            if (autoAimEnabled) {
                double desiredTurretAngle = robot.calculateTurretAutoAimAngle(
                        TARGET_GOAL_X, TARGET_GOAL_Y, TURRET_OFFSET_X, TURRET_OFFSET_Y);

                double kP = 0.025;
                turretPower = Math.min(Math.max(desiredTurretAngle * kP, -0.8), 0.8);
            } else {
                turretPower = gamepad2.left_stick_x;
                if (Math.abs(turretPower) < 0.1) {
                    if (gamepad2.dpad_left) {
                        turretPower = -0.4;
                    } else if (gamepad2.dpad_right) {
                        turretPower = 0.4;
                    } else {
                        turretPower = 0.0;
                    }
                }
            }

            if (robot.turretMotor != null) {
                robot.turretMotor.setPower(turretPower);
            }

            // ----------------------------------------------------
            // 3. SHOOTER FLYWHEEL & BLOCKER SERVO
            // ----------------------------------------------------
            double shooterPower = gamepad2.right_trigger;
            if (gamepad2.a) {
                shooterPower = 0.85; // Preset flywheel shooting power
            }
            if (robot.shooterMotor != null) {
                robot.shooterMotor.setPower(shooterPower);
            }

            // Blocker Servo: Open when shooting or holding Gamepad 2 Right Bumper
            if (shooterPower > 0.2 || gamepad2.right_bumper) {
                robot.openBlocker();
            } else {
                robot.closeBlocker();
            }

            // ----------------------------------------------------
            // 4. FLOWER MECHANISM SERVO (FRONT WEDGE)
            // ----------------------------------------------------
            if (gamepad2.dpad_up) {
                robot.deployFlower();
            } else if (gamepad2.dpad_down) {
                robot.retractFlower();
            }

            // ----------------------------------------------------
            // 5. SPLIT INTAKE & TRANSFER MOTOR CONTROLS
            // ----------------------------------------------------
            double intakePower = 0.0;
            double transferPower = 0.0;

            if (gamepad2.x) {
                // Spin-In-Place / Anti-Stall Mode:
                intakePower = 0.7;
                transferPower = -0.7;
            } else if (gamepad2.y) {
                // Feed transfer into shooter only
                intakePower = 0.0;
                transferPower = 0.9;
            } else if (gamepad2.left_trigger > 0.1) {
                // Main Intake Forward
                intakePower = gamepad2.left_trigger;
                transferPower = gamepad2.left_trigger;
            } else if (gamepad2.left_bumper) {
                // Reverse Both
                intakePower = -0.7;
                transferPower = -0.7;
            }

            if (robot.intakeMotor != null) {
                robot.intakeMotor.setPower(intakePower);
            }
            if (robot.transferMotor != null) {
                robot.transferMotor.setPower(transferPower);
            }

            // ----------------------------------------------------
            // 6. TELEMETRY & LOCALIZATION
            // ----------------------------------------------------
            telemetry.addData("--- GoBILDA Pinpoint Pose ---", "X:%.1f in  Y:%.1f in  Heading:%.1f deg",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Turret Mode", autoAimEnabled ? "AUTO-AIMING" : "MANUAL");

            List<AprilTagDetection> detections = robot.getAprilTagDetections();
            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    if (detection instanceof AprilTagSingleDetection) {
                        AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                        telemetry.addData("AprilTag Detected", "ID %d", singleDet.id);
                        break;
                    }
                }
            }

            telemetry.addData("Servos", "Blocker:%s  Flower:%s",
                    (robot.blockerServo != null ? String.format("%.2f", robot.blockerServo.getPosition()) : "N/A"),
                    (robot.flowerServo != null ? String.format("%.2f", robot.flowerServo.getPosition()) : "N/A"));
            telemetry.update();
        }

        robot.closeVision();
    }
}
