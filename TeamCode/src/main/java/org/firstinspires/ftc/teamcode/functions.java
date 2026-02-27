package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.shooter.ShooterData;

public class functions {
    public static final double SERVO_MAX_ANGLE_DEGREES = 300.0;

    public static final double FLYWHEEL_P = 3.0;
    public static final double FLYWHEEL_I = 0.0;
    public static final double FLYWHEEL_D = 0.0;
    public static final double FLYWHEEL_F = 7.0;

    public static final double RAMP_WHEEL_P = 4.0;
    public static final double RAMP_WHEEL_I = 0.0;
    public static final double RAMP_WHEEL_D = 0.0;
    public static final double RAMP_WHEEL_F = 20.0;

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

    public static PIDFCoefficients newRampWheelPidf() {
        return new PIDFCoefficients(RAMP_WHEEL_P, RAMP_WHEEL_I, RAMP_WHEEL_D, RAMP_WHEEL_F);
    }

    public static void applyFlywheelPidf(DcMotorEx leftFlyWheel, DcMotorEx rightFlyWheel) {
        if (leftFlyWheel == null || rightFlyWheel == null) return;
        PIDFCoefficients pidf = newFlywheelPidf();
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public static void applyRampWheelPidf(DcMotorEx rampWheelMotor) {
        if (rampWheelMotor == null) return;
        rampWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newRampWheelPidf());
    }

    public static void setDualVelocity(DcMotorEx motorA, DcMotorEx motorB, double velocity) {
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

        PIDFCoefficients intakePidf = new PIDFCoefficients(0, 0, 0, 0);
        rightintake.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, intakePidf);
        leftintake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, intakePidf);
    }

    public static class TimedShotSequence {
        private final ElapsedTime timer = new ElapsedTime();
        private final double indexerUpAngleDegrees;
        private final double indexerDownAngleDegrees;
        private final double rampVelocity;
        private final double indexerDownStartSec;
        private final double stopRampAtSec;

        private boolean active = false;
        private double currentIndexerTargetDegrees;

        public TimedShotSequence(double indexerUpAngleDegrees,
                                 double indexerDownAngleDegrees,
                                 double rampVelocity,
                                 double indexerDownStartSec,
                                 double stopRampAtSec) {
            this.indexerUpAngleDegrees = indexerUpAngleDegrees;
            this.indexerDownAngleDegrees = indexerDownAngleDegrees;
            this.rampVelocity = rampVelocity;
            this.indexerDownStartSec = indexerDownStartSec;
            this.stopRampAtSec = stopRampAtSec;
            this.currentIndexerTargetDegrees = indexerDownAngleDegrees;
        }

        public void trigger(DcMotorEx rampWheelMotor) {
            applyRampWheelPidf(rampWheelMotor);
            if (rampWheelMotor != null) rampWheelMotor.setVelocity(rampVelocity);
            currentIndexerTargetDegrees = indexerUpAngleDegrees;
            timer.reset();
            active = true;
        }

        public void update(DcMotorEx rampWheelMotor, Servo leftIndexer, Servo rightIndexer) {
            if (active) {
                double elapsed = timer.seconds();
                if (elapsed >= indexerDownStartSec) {
                    currentIndexerTargetDegrees = indexerDownAngleDegrees;
                }
                if (elapsed >= stopRampAtSec) {
                    if (rampWheelMotor != null) rampWheelMotor.setVelocity(0.0);
                    active = false;
                }
            }

            setDualServoAngle(leftIndexer, rightIndexer, currentIndexerTargetDegrees);
        }

        public void resetToDown(Servo leftIndexer, Servo rightIndexer) {
            active = false;
            currentIndexerTargetDegrees = indexerDownAngleDegrees;
            setDualServoAngle(leftIndexer, rightIndexer, currentIndexerTargetDegrees);
        }

        public boolean isActive() {
            return active;
        }

        public double getCurrentIndexerTargetDegrees() {
            return currentIndexerTargetDegrees;
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

        public Double getSuggestedShooterDistanceInches() {
            // TODO: Convert MegaTag2 robot pose to distance from a field target.
            return null;
        }

        public void addTelemetry(Telemetry telemetry) {
            Telemetry out = telemetry != null ? telemetry : fallbackTelemetry;
            if (out == null) return;
            out.addData("Limelight MT2", status);
            if (latestResult != null) {
                out.addData("LL tx", "%.2f", latestResult.getTx());
                out.addData("LL ty", "%.2f", latestResult.getTy());
                out.addData("LL ta", "%.2f", latestResult.getTa());
            }
        }
    }
}
