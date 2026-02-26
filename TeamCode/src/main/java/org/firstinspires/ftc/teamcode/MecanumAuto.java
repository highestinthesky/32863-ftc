package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.BezierLine;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "MecanumAuto", group = "Test")
public class MecanumAuto extends LinearOpMode {

    private static final double START_X = 72;
    private static final double START_Y = 72;
    private static final double FORWARD_DISTANCE_IN = 24;
    private static final double PATH_TIMEOUT_SEC = 5.0;

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);

        Pose startPose = new Pose(START_X, START_Y);
        Pose endPose = new Pose(START_X + FORWARD_DISTANCE_IN, START_Y);

        follower.setStartingPose(startPose);

        Path forwardPath = new Path(new BezierLine(startPose, endPose));
        forwardPath.setConstantHeadingInterpolation(0);

        telemetry.addLine("Pedro auto ready. Press PLAY.");
        telemetry.addData("Start", "(%.1f, %.1f)", START_X, START_Y);
        telemetry.addData("End", "(%.1f, %.1f)", START_X + FORWARD_DISTANCE_IN, START_Y);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        follower.followPath(forwardPath);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            Pose pose = follower.getPose();
            telemetry.addData("Busy", follower.isBusy());
            telemetry.addData("Time", "%.2f / %.2f", timer.seconds(), PATH_TIMEOUT_SEC);
            telemetry.addData("Pose", "x=%.2f y=%.2f h=%.2f", pose.getX(), pose.getY(), pose.getHeading());
            telemetry.update();

            if (!follower.isBusy()) break;
            if (timer.seconds() > PATH_TIMEOUT_SEC) {
                telemetry.addLine("Path timeout reached");
                telemetry.update();
                break;
            }
        }

        // Keep localizer updates alive briefly while the op mode remains active.
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            Pose pose = follower.getPose();
            telemetry.addLine("Path complete");
            telemetry.addData("Pose", "x=%.2f y=%.2f h=%.2f", pose.getX(), pose.getY(), pose.getHeading());
            telemetry.update();
            idle();
        }
    }
}
