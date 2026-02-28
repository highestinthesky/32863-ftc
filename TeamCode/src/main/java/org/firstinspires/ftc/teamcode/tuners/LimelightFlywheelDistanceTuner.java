package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.functions;
import org.firstinspires.ftc.teamcode.shooter.ShooterData;

@TeleOp(name = "LimelightFlywheelDistanceTuner", group = "Tuners")
public class LimelightFlywheelDistanceTuner extends OpMode {
    private static final int LIMELIGHT_APRILTAG_PIPELINE = 1;
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;

    private final double[] stepSizes = {1000.0, 500.0, 100.0, 10.0, 1.0};
    private int stepIndex = 2;

    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;
    private functions.MegaTag2Prep megaTag2;
    private final ShooterData shooterData = ShooterData.defaultTuned();

    private int goalTagId = BLUE_GOAL_TAG_ID;
    private boolean spinEnabled = false;
    private double manualVelocity = 1800.0;

    private Double lastValidDistanceInches = null;
    private Double lastValidTxDegrees = null;

    @Override
    public void init() {
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");

        rightFlyWheel.setDirection(DcMotorEx.Direction.FORWARD);
        leftFlyWheel.setDirection(DcMotorEx.Direction.REVERSE);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        functions.applyFlywheelPidf(leftFlyWheel, rightFlyWheel);
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        megaTag2 = new functions.MegaTag2Prep(hardwareMap, telemetry, "limelight", "imu", LIMELIGHT_APRILTAG_PIPELINE);
        megaTag2.initializeAndConfigure();

        telemetry.addLine("LimelightFlywheelDistanceTuner ready");
        telemetry.addLine("Y=spin, B=step, Dpad L/R=velocity, A=tag");
        telemetry.update();
    }

    @Override
    public void start() {
        if (megaTag2 != null) megaTag2.start();
    }

    @Override
    public void loop() {
        if (megaTag2 != null) megaTag2.update();

        if (gamepad1.yWasPressed()) spinEnabled = !spinEnabled;
        if (gamepad1.bWasPressed()) stepIndex = (stepIndex + 1) % stepSizes.length;
        if (gamepad1.dpadLeftWasPressed()) manualVelocity = Math.max(0.0, manualVelocity - stepSizes[stepIndex]);
        if (gamepad1.dpadRightWasPressed()) manualVelocity += stepSizes[stepIndex];
        if (gamepad1.aWasPressed()) goalTagId = (goalTagId == BLUE_GOAL_TAG_ID) ? RED_GOAL_TAG_ID : BLUE_GOAL_TAG_ID;

        Double distanceInches = megaTag2 != null ? megaTag2.getGoalTagHorizontalDistanceInches(goalTagId) : null;
        Double txDegrees = megaTag2 != null ? megaTag2.getGoalTagTxDegrees(goalTagId) : null;
        if (distanceInches != null) lastValidDistanceInches = distanceInches;
        if (txDegrees != null) lastValidTxDegrees = txDegrees;

        // This tuner is intended for manually finding velocity for each measured distance.
        double interpolatedVelocity = functions.computeShooterVelocityTarget(shooterData, distanceInches, manualVelocity);
        double commandVelocity = manualVelocity;

        if (spinEnabled) {
            leftFlyWheel.setVelocity(commandVelocity);
            rightFlyWheel.setVelocity(commandVelocity);
        } else {
            leftFlyWheel.setVelocity(0.0);
            rightFlyWheel.setVelocity(0.0);
        }

        double leftVelocity = leftFlyWheel.getVelocity();
        double rightVelocity = rightFlyWheel.getVelocity();
        double totalCurrent = leftFlyWheel.getCurrent(CurrentUnit.AMPS) + rightFlyWheel.getCurrent(CurrentUnit.AMPS);

        telemetry.addLine("----- Limelight + Flywheel Distance Tuner -----");
        telemetry.addData("Goal Tag ID", goalTagId);
        telemetry.addData("Tag Status", megaTag2 != null ? megaTag2.getGoalTagStatus(goalTagId) : "n/a");
        telemetry.addData("Pipeline Check", megaTag2 != null ? megaTag2.getPipelineValidationMessage() : "n/a");
        telemetry.addData("Distance (in) [LIVE]", distanceInches == null ? "n/a" : String.format("%.2f", distanceInches));
        telemetry.addData("tx (deg) [LIVE]", txDegrees == null ? "n/a" : String.format("%.2f", txDegrees));
        telemetry.addData("Distance (in) [LAST]", lastValidDistanceInches == null ? "n/a" : String.format("%.2f", lastValidDistanceInches));
        telemetry.addData("tx (deg) [LAST]", lastValidTxDegrees == null ? "n/a" : String.format("%.2f", lastValidTxDegrees));

        telemetry.addData("Spin Enabled", spinEnabled);
        telemetry.addData("Step Size", "%.1f", stepSizes[stepIndex]);
        telemetry.addData("Manual Velocity", "%.1f", manualVelocity);
        telemetry.addData("ShooterData Suggestion", "%.1f", interpolatedVelocity);
        telemetry.addData("Command Velocity", "%.1f", commandVelocity);
        telemetry.addData("Left Wheel Velocity", "%.1f", leftVelocity);
        telemetry.addData("Right Wheel Velocity", "%.1f", rightVelocity);
        telemetry.addData("Total Current (A)", "%.2f", totalCurrent);

        Double distanceForTemplate = distanceInches != null ? distanceInches : lastValidDistanceInches;
        if (distanceForTemplate != null) {
            telemetry.addLine("ShooterData template (paste after tuning this shot):");
            telemetry.addData("Template", "data.put(%.1f, %.0f);", distanceForTemplate, commandVelocity);
        }

        if (megaTag2 != null) {
            megaTag2.addTelemetry(telemetry);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        leftFlyWheel.setVelocity(0.0);
        rightFlyWheel.setVelocity(0.0);
    }
}
