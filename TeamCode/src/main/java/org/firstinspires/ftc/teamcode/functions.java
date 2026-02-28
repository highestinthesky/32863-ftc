package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.shooter.ShooterData;

import java.util.List;

public class functions {
    public static final double SERVO_MAX_ANGLE_DEGREES = 300.0;

    public static final double FLYWHEEL_P = 3.0;
    public static final double FLYWHEEL_I = 0.0;
    public static final double FLYWHEEL_D = 0.0;
    public static final double FLYWHEEL_F = 7.0;

    public static final double INTAKE_P = 6.0;
    public static final double INTAKE_I = 0.0;
    public static final double INTAKE_D = 0.0;
    public static final double INTAKE_F = 4.0;

    public static double getMotorExTPS(DcMotorEx motor) {
        return motor.getVelocity();
    }

    public static boolean isTPSLow(DcMotorEx motor, double expectedTPS) {
        return motor.getVelocity() < expectedTPS - 200;
    }

    public static void consecutiveShots(DcMotorEx flywheel1, DcMotorEx flywheel2, double expectedTPS) {
        double curTPS1 = getMotorExTPS(flywheel1);
        double curTPS2 = getMotorExTPS(flywheel2);
        if (curTPS1 < 0 || curTPS2 < 0 || expectedTPS < 0) {
            // placeholder for future shooter sequencing logic
        }
    }

    public static PIDFCoefficients newFlywheelPidf() {
        return new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
    }

    public static PIDFCoefficients newIntakePidf() {
        return new PIDFCoefficients(INTAKE_P, INTAKE_I, INTAKE_D, INTAKE_F);
    }

    public static void applyFlywheelPidf(DcMotorEx leftFlyWheel, DcMotorEx rightFlyWheel) {
        if (leftFlyWheel == null || rightFlyWheel == null) return;
        PIDFCoefficients pidf = newFlywheelPidf();
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public static void applyIntakePidf(DcMotorEx intakeMotor) {
        if (intakeMotor == null) return;
        intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newIntakePidf());
    }

    public static void setTwoMotorsVelocity(DcMotorEx motorA, DcMotorEx motorB, double velocity) {
        if (motorA != null) motorA.setVelocity(velocity);
        if (motorB != null) motorB.setVelocity(velocity);
    }

    public static void setDualPower(DcMotorEx motorA, DcMotorEx motorB, double power) {
        if (motorA != null) motorA.setPower(power);
        if (motorB != null) motorB.setPower(power);
    }

    public static void setServoAngle(Servo servo, double angleDegrees) {
        if (servo == null) return;
        servo.setPosition(clamp(angleDegrees / SERVO_MAX_ANGLE_DEGREES, 0.0, 1.0));
    }

    public static void setDualServoAngle(Servo servoA, Servo servoB, double angleDegrees) {
        setServoAngle(servoA, angleDegrees);
        setServoAngle(servoB, angleDegrees);
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double computeShooterVelocityTarget(ShooterData shooterData, Double distanceInches, double fallbackVelocity) {
        if (distanceInches == null || shooterData == null || shooterData.isEmpty()) return fallbackVelocity;
        return shooterData.getTargetVelocity(distanceInches);
    }

    public static void setupMotors(
            DcMotorEx leftFrontDrive,
            DcMotorEx rightFrontDrive,
            DcMotorEx leftBackDrive,
            DcMotorEx rightBackDrive,
            DcMotorEx leftFlyWheel,
            DcMotorEx rightFlyWheel,
            DcMotorEx rightintake,
            DcMotorEx leftintake) {

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFlyWheel.setDirection(DcMotorEx.Direction.FORWARD);
        leftFlyWheel.setDirection(DcMotorEx.Direction.REVERSE);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyFlywheelPidf(leftFlyWheel, rightFlyWheel);

        rightintake.setDirection(DcMotorEx.Direction.FORWARD);
        leftintake.setDirection(DcMotorEx.Direction.FORWARD);
        rightintake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftintake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients intakePidf = newIntakePidf();
        rightintake.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, intakePidf);
        leftintake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, intakePidf);
    }

    public static class TimedIntakePulse {
        private final ElapsedTime timer = new ElapsedTime();
        private final double intakeVelocity;
        private final double stopAtSec;

        private boolean active = false;

        public TimedIntakePulse(double intakeVelocity, double stopAtSec) {
            this.intakeVelocity = intakeVelocity;
            this.stopAtSec = stopAtSec;
        }

        public void trigger(DcMotorEx intakeMotor) {
            if (intakeMotor != null) intakeMotor.setVelocity(intakeVelocity);
            timer.reset();
            active = true;
        }

        public void update(DcMotorEx intakeMotor) {
            if (active) {
                double elapsed = timer.seconds();
                if (elapsed >= stopAtSec) {
                    if (intakeMotor != null) intakeMotor.setVelocity(0.0);
                    active = false;
                }
            }
        }

        public boolean isActive() {
            return active;
        }

        public double getElapsedSeconds() {
            return active ? timer.seconds() : 0.0;
        }
    }

    public static class PinpointIO {
        private final GoBildaPinpointDriver odo;
        private final Telemetry telemetry;

        public PinpointIO(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
            this.telemetry = telemetry;
            this.odo = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
        }

        public void initializeAndConfigure() {
            odo.resetPosAndIMU();
            if (telemetry != null) {
                telemetry.addLine("Pinpoint initialized");
                telemetry.update();
            }
        }

        public void update() {
            odo.update();
        }

        public double getXInches() {
            return odo.getPosition().getX(DistanceUnit.INCH);
        }

        public double getYInches() {
            return odo.getPosition().getY(DistanceUnit.INCH);
        }

        public double getHeadingRadians() {
            return odo.getPosition().getHeading(AngleUnit.RADIANS);
        }
    }

    public static class MegaTag2Prep {
        private final Telemetry fallbackTelemetry;
        private final int pipelineId;

        private Limelight3A limelight;
        private IMU imu;
        private LLResult latestResult;
        private Pose3D latestMegaTag2Pose;
        private boolean available;
        private String status = "Disabled";
        private String pipelineType = "unknown";
        private int activePipelineIndex = -1;
        private boolean connected = false;
        private double fps = 0.0;

        public MegaTag2Prep(HardwareMap hardwareMap, Telemetry telemetry, String limelightName, String imuName, int pipelineId) {
            this.fallbackTelemetry = telemetry;
            this.pipelineId = pipelineId;
            try {
                limelight = hardwareMap.get(Limelight3A.class, limelightName);
                imu = hardwareMap.get(IMU.class, imuName);
                available = true;
                status = "Bound";
            } catch (Exception e) {
                available = false;
                status = "Unavailable: " + e.getClass().getSimpleName();
            }
        }

        public void initializeAndConfigure() {
            if (!available) return;
            try {
                limelight.pipelineSwitch(pipelineId);
                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                status = "Initialized";
            } catch (Exception e) {
                status = "Init error: " + e.getClass().getSimpleName();
            }
        }

        public void start() {
            if (!available) return;
            try {
                limelight.start();
                status = "Running";
            } catch (Exception e) {
                status = "Start error: " + e.getClass().getSimpleName();
            }
        }

        public void update() {
            latestResult = null;
            latestMegaTag2Pose = null;

            if (!available) return;

            try {
                LLStatus limelightStatus = limelight.getStatus();
                connected = limelight.isConnected();
                if (limelightStatus != null) {
                    pipelineType = limelightStatus.getPipelineType();
                    activePipelineIndex = limelightStatus.getPipelineIndex();
                    fps = limelightStatus.getFps();
                }

                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                limelight.updateRobotOrientation(orientation.getYaw());
                latestResult = limelight.getLatestResult();
                if (latestResult != null && latestResult.isValid()) {
                    latestMegaTag2Pose = latestResult.getBotpose_MT2();
                    status = "Valid MT2";
                } else {
                    status = "No valid result";
                }
            } catch (Exception e) {
                status = "Update error: " + e.getClass().getSimpleName();
            }
        }

        public boolean isAvailable() {
            return available;
        }

        public boolean hasValidResult() {
            return latestResult != null && latestResult.isValid();
        }

        public LLResult getLatestResult() {
            return latestResult;
        }

        public Pose3D getLatestMegaTag2Pose() {
            return latestMegaTag2Pose;
        }

        public String getStatus() {
            return status;
        }

        public String getPipelineType() {
            return pipelineType;
        }

        public int getActivePipelineIndex() {
            return activePipelineIndex;
        }

        public boolean isConnected() {
            return connected;
        }

        public double getFps() {
            return fps;
        }

        public boolean isAprilTagPipelineLikely() {
            if (pipelineType == null) return false;
            String normalized = pipelineType.toLowerCase();
            return normalized.contains("apriltag")
                    || normalized.contains("fiducial")
                    || normalized.contains("tag");
        }

        public String getPipelineValidationMessage() {
            if (!connected) return "Limelight disconnected";
            if (activePipelineIndex != pipelineId) {
                return String.format("Wrong pipeline active: %d (expected %d)", activePipelineIndex, pipelineId);
            }
            if (!isAprilTagPipelineLikely()) {
                return "Active pipeline is not AprilTag/Fiducial";
            }
            return "Pipeline OK for AprilTag";
        }

        public Double getSuggestedShooterDistanceInches() {
            LLResultTypes.FiducialResult fiducial = getFirstFiducial();
            return computeHorizontalDistanceInches(fiducial);
        }

        public Double getGoalTagHorizontalDistanceInches(int goalTagId) {
            LLResultTypes.FiducialResult fiducial = getFiducialById(goalTagId);
            return computeHorizontalDistanceInches(fiducial);
        }

        public Double getGoalTagTxDegrees(int goalTagId) {
            LLResultTypes.FiducialResult fiducial = getFiducialById(goalTagId);
            if (fiducial == null) return null;
            return fiducial.getTargetXDegrees();
        }

        public boolean hasGoalTag(int goalTagId) {
            return getFiducialById(goalTagId) != null;
        }

        public String getGoalTagStatus(int goalTagId) {
            if (!available) return "Limelight unavailable";
            if (latestResult == null) return "No Limelight result";
            if (!latestResult.isValid()) return "Invalid Limelight result";

            LLResultTypes.FiducialResult fiducial = getFiducialById(goalTagId);
            if (fiducial == null) return "Goal tag " + goalTagId + " not found";

            Double distanceInches = computeHorizontalDistanceInches(fiducial);
            if (distanceInches == null) return "Goal tag " + goalTagId + " pose unavailable";

            return String.format("Goal tag %d found (%.1f in)", goalTagId, distanceInches);
        }

        private LLResultTypes.FiducialResult getFirstFiducial() {
            if (latestResult == null || !latestResult.isValid()) return null;
            List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) return null;
            return fiducials.get(0);
        }

        private LLResultTypes.FiducialResult getFiducialById(int goalTagId) {
            if (latestResult == null || !latestResult.isValid()) return null;
            List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) return null;

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial != null && fiducial.getFiducialId() == goalTagId) {
                    return fiducial;
                }
            }
            return null;
        }

        private Double computeHorizontalDistanceInches(LLResultTypes.FiducialResult fiducial) {
            if (fiducial == null) return null;
            try {
                Pose3D targetPose = fiducial.getTargetPoseCameraSpace();
                if (targetPose == null) return null;
                Position cameraSpacePosition = targetPose.getPosition();
                if (cameraSpacePosition == null) return null;

                Position inchesPosition = cameraSpacePosition.toUnit(DistanceUnit.INCH);
                double x = inchesPosition.x;
                double z = inchesPosition.z;
                double distance = Math.hypot(x, z);
                if (Double.isNaN(distance) || Double.isInfinite(distance)) return null;
                return distance;
            } catch (Exception ignored) {
                return null;
            }
        }

        public void addTelemetry(Telemetry telemetry) {
            Telemetry out = telemetry != null ? telemetry : fallbackTelemetry;
            if (out == null) return;
            out.addData("Limelight MT2", status);
            out.addData("LL Connected", connected);
            out.addData("LL Active Pipeline", activePipelineIndex);
            out.addData("LL Pipeline Type", pipelineType);
            out.addData("LL Pipeline Check", getPipelineValidationMessage());
            out.addData("LL FPS", "%.1f", fps);
            if (latestResult != null) {
                out.addData("LL tx", "%.2f", latestResult.getTx());
                out.addData("LL ty", "%.2f", latestResult.getTy());
                out.addData("LL ta", "%.2f", latestResult.getTa());
            }
        }
    }
}
