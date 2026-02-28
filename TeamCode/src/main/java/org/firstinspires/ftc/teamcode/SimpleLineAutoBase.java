package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Shared minimal Pedro-path auto used by Near/Far alliance variants.
 * This is intentionally simple so each variant can be tuned quickly.
 */
abstract class SimpleLineAutoBase extends LinearOpMode {
    protected static final class Config {
        final String label;
        final double startX;
        final double startY;
        final double forwardDistanceIn;
        final double timeoutSec;

        private Config(String label, double startX, double startY, double forwardDistanceIn, double timeoutSec) {
            this.label = label;
            this.startX = startX;
            this.startY = startY;
            this.forwardDistanceIn = forwardDistanceIn;
            this.timeoutSec = timeoutSec;
        }
    }

    protected static Config config(String label, double startX, double startY, double forwardDistanceIn, double timeoutSec) {
        return new Config(label, startX, startY, forwardDistanceIn, timeoutSec);
    }

    protected abstract Config getConfig();

    @Override
    public void runOpMode() {
        Config cfg = getConfig();
        Follower follower = Constants.createFollower(hardwareMap);

        Pose startPose = new Pose(cfg.startX, cfg.startY);
        Pose endPose = new Pose(cfg.startX + cfg.forwardDistanceIn, cfg.startY);

        follower.setStartingPose(startPose);

        Path forwardPath = new Path(new BezierLine(startPose, endPose));
        forwardPath.setConstantHeadingInterpolation(0);

        telemetry.addLine(cfg.label + " ready. Press PLAY.");
        telemetry.addData("Start", "(%.1f, %.1f)", cfg.startX, cfg.startY);
        telemetry.addData("End", "(%.1f, %.1f)", cfg.startX + cfg.forwardDistanceIn, cfg.startY);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        follower.followPath(forwardPath);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            Pose pose = follower.getPose();
            telemetry.addData("Auto", cfg.label);
            telemetry.addData("Busy", follower.isBusy());
            telemetry.addData("Time", "%.2f / %.2f", timer.seconds(), cfg.timeoutSec);
            telemetry.addData("Pose", "x=%.2f y=%.2f h=%.2f", pose.getX(), pose.getY(), pose.getHeading());
            telemetry.update();

            if (!follower.isBusy()) break;
            if (timer.seconds() > cfg.timeoutSec) {
                telemetry.addLine("Path timeout reached");
                telemetry.update();
                break;
            }
        }

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            Pose pose = follower.getPose();
            telemetry.addData("Auto", cfg.label);
            telemetry.addLine("Path complete");
            telemetry.addData("Pose", "x=%.2f y=%.2f h=%.2f", pose.getX(), pose.getY(), pose.getHeading());
            telemetry.update();
            idle();
        }
    }
}
