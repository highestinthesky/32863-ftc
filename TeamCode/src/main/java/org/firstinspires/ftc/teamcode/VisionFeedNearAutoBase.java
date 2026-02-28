package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooter.ShooterData;
import org.firstinspires.ftc.teamcode.shooter.ShotControlConfig;
import org.firstinspires.ftc.teamcode.shooter.VisionShooterTargeting;

/**
 * Near-side autonomous:
 * 1) Move backward a fixed distance.
 * 2) Use Limelight to find alliance goal tag and compute flywheel target velocity.
 * 3) Run both intakes to feed balls into the flywheel.
 */
abstract class VisionFeedNearAutoBase extends LinearOpMode {
    protected static final class Config {
        final String label;
        final double startX;
        final double startY;
        final double backDistanceIn;
        final double pathTimeoutSec;
        final int goalTagId;
        final int limelightPipelineId;
        final double tagAcquireTimeoutSec;
        final double flywheelSpinupTimeoutSec;
        final double feedDurationSec;

        private Config(
                String label,
                double startX,
                double startY,
                double backDistanceIn,
                double pathTimeoutSec,
                int goalTagId,
                int limelightPipelineId,
                double tagAcquireTimeoutSec,
                double flywheelSpinupTimeoutSec,
                double feedDurationSec
        ) {
            this.label = label;
            this.startX = startX;
            this.startY = startY;
            this.backDistanceIn = backDistanceIn;
            this.pathTimeoutSec = pathTimeoutSec;
            this.goalTagId = goalTagId;
            this.limelightPipelineId = limelightPipelineId;
            this.tagAcquireTimeoutSec = tagAcquireTimeoutSec;
            this.flywheelSpinupTimeoutSec = flywheelSpinupTimeoutSec;
            this.feedDurationSec = feedDurationSec;
        }
    }

    protected static Config config(
            String label,
            double startX,
            double startY,
            double backDistanceIn,
            double pathTimeoutSec,
            int goalTagId,
            int limelightPipelineId,
            double tagAcquireTimeoutSec,
            double flywheelSpinupTimeoutSec,
            double feedDurationSec
    ) {
        return new Config(
                label,
                startX,
                startY,
                backDistanceIn,
                pathTimeoutSec,
                goalTagId,
                limelightPipelineId,
                tagAcquireTimeoutSec,
                flywheelSpinupTimeoutSec,
                feedDurationSec
        );
    }

    protected abstract Config getConfig();

    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;
    private DcMotorEx rightIntake;
    private DcMotorEx leftIntake;

    private functions.MegaTag2Prep megaTag2;
    private final ShooterData shooterData = ShooterData.defaultTuned();
    private final ShotControlConfig shotConfig = ShotControlConfig.defaultConfig();

    @Override
    public void runOpMode() {
        Config cfg = getConfig();
        Follower follower = Constants.createFollower(hardwareMap);

        initializeMechanisms();

        megaTag2 = new functions.MegaTag2Prep(hardwareMap, telemetry, "limelight", "imu", cfg.limelightPipelineId);
        megaTag2.initializeAndConfigure();

        Pose startPose = new Pose(cfg.startX, cfg.startY);
        Pose endPose = new Pose(cfg.startX - Math.abs(cfg.backDistanceIn), cfg.startY);
        follower.setStartingPose(startPose);

        Path backwardPath = new Path(new BezierLine(startPose, endPose));
        backwardPath.setConstantHeadingInterpolation(0);

        telemetry.addData("Auto", cfg.label);
        telemetry.addData("Move", "Back %.1f in", Math.abs(cfg.backDistanceIn));
        telemetry.addData("Goal Tag", cfg.goalTagId);
        telemetry.addLine("Ready. Press PLAY.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        megaTag2.start();

        followPathWithTimeout(follower, backwardPath, cfg);
        if (opModeIsActive() && !isStopRequested()) {
            runVisionFeedSequence(cfg);
        }

        stopMechanisms();
    }

    private void initializeMechanisms() {
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightintake");
        leftIntake = hardwareMap.get(DcMotorEx.class, "leftintake");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        functions.applyFlywheelPidf(leftFlyWheel, rightFlyWheel);
        functions.applyIntakePidf(rightIntake);
        functions.applyIntakePidf(leftIntake);

        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopMechanisms();
    }

    private void followPathWithTimeout(Follower follower, Path path, Config cfg) {
        follower.followPath(path);
        ElapsedTime pathTimer = new ElapsedTime();
        pathTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            megaTag2.update();

            Pose pose = follower.getPose();
            telemetry.addData("Auto", cfg.label);
            telemetry.addData("Step", "Moving back");
            telemetry.addData("Busy", follower.isBusy());
            telemetry.addData("Time", "%.2f / %.2f", pathTimer.seconds(), cfg.pathTimeoutSec);
            telemetry.addData("Pose", "x=%.2f y=%.2f h=%.2f", pose.getX(), pose.getY(), pose.getHeading());
            telemetry.update();

            if (!follower.isBusy()) break;
            if (pathTimer.seconds() > cfg.pathTimeoutSec) break;
            idle();
        }
    }

    private void runVisionFeedSequence(Config cfg) {
        VisionShooterTargeting targeting = new VisionShooterTargeting(megaTag2, shooterData, cfg.goalTagId);

        Double targetDistance = null;
        double targetVelocity = 0.0;

        ElapsedTime acquireTimer = new ElapsedTime();
        acquireTimer.reset();
        while (opModeIsActive() && !isStopRequested() && acquireTimer.seconds() < cfg.tagAcquireTimeoutSec) {
            megaTag2.update();
            targeting.update();

            telemetry.addData("Auto", cfg.label);
            telemetry.addData("Step", "Finding goal tag");
            telemetry.addData("Goal Tag", cfg.goalTagId);
            telemetry.addData("Vision", targeting.getStatus());
            telemetry.addData("Acquire", "%.2f / %.2f", acquireTimer.seconds(), cfg.tagAcquireTimeoutSec);
            telemetry.update();

            if (targeting.isTargetAvailable()) {
                targetDistance = targeting.getDistanceInches();
                targetVelocity = Math.max(0.0, targeting.getTargetVelocity());
                break;
            }
            idle();
        }

        if (targetVelocity <= 0.0) {
            telemetry.addData("Auto", cfg.label);
            telemetry.addLine("Goal tag not found. Feed canceled.");
            telemetry.update();
            return;
        }

        spinupFlywheel(targetVelocity, cfg, targetDistance);
        feedAllIntakes(targetVelocity, cfg, targetDistance);
    }

    private void spinupFlywheel(double targetVelocity, Config cfg, Double targetDistance) {
        functions.setTwoMotorsVelocity(leftFlyWheel, rightFlyWheel, targetVelocity);

        ElapsedTime spinupTimer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        boolean settleActive = false;
        spinupTimer.reset();

        while (opModeIsActive() && !isStopRequested() && spinupTimer.seconds() < cfg.flywheelSpinupTimeoutSec) {
            double leftSpeed = Math.abs(leftFlyWheel.getVelocity());
            double rightSpeed = Math.abs(rightFlyWheel.getVelocity());
            double target = Math.abs(targetVelocity);

            boolean ready = Math.abs(leftSpeed - target) <= shotConfig.flywheelReadyTolerance
                    && Math.abs(rightSpeed - target) <= shotConfig.flywheelReadyTolerance;

            if (ready) {
                if (!settleActive) {
                    settleActive = true;
                    settleTimer.reset();
                }
            } else {
                settleActive = false;
            }

            telemetry.addData("Auto", cfg.label);
            telemetry.addData("Step", "Spinning flywheel");
            telemetry.addData("Goal Tag", cfg.goalTagId);
            telemetry.addData("Distance (in)", targetDistance == null ? "n/a" : String.format("%.1f", targetDistance));
            telemetry.addData("Target Vel", "%.1f", targetVelocity);
            telemetry.addData("Measured L/R", "%.0f / %.0f", leftSpeed, rightSpeed);
            telemetry.addData("Spinup", "%.2f / %.2f", spinupTimer.seconds(), cfg.flywheelSpinupTimeoutSec);
            telemetry.update();

            if (settleActive && settleTimer.seconds() >= shotConfig.flywheelReadySettleSeconds) {
                break;
            }
            idle();
        }
    }

    private void feedAllIntakes(double targetVelocity, Config cfg, Double targetDistance) {
        ElapsedTime feedTimer = new ElapsedTime();
        feedTimer.reset();
        while (opModeIsActive() && !isStopRequested() && feedTimer.seconds() < cfg.feedDurationSec) {
            // Hold flywheel speed while feeding.
            functions.setTwoMotorsVelocity(leftFlyWheel, rightFlyWheel, targetVelocity);
            // "All intakes" inward to transfer into flywheel.
            rightIntake.setPower(1.0);
            leftIntake.setPower(1.0);

            telemetry.addData("Auto", cfg.label);
            telemetry.addData("Step", "Feeding all intakes");
            telemetry.addData("Goal Tag", cfg.goalTagId);
            telemetry.addData("Distance (in)", targetDistance == null ? "n/a" : String.format("%.1f", targetDistance));
            telemetry.addData("Target Vel", "%.1f", targetVelocity);
            telemetry.addData("Feed", "%.2f / %.2f", feedTimer.seconds(), cfg.feedDurationSec);
            telemetry.update();
            idle();
        }
    }

    private void stopMechanisms() {
        functions.setTwoMotorsVelocity(leftFlyWheel, rightFlyWheel, 0.0);
        functions.setDualPower(rightIntake, leftIntake, 0.0);
    }
}
