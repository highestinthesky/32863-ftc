package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.functions;

@TeleOp(name = "LimelightFlywheelDistanceTuner", group = "Tuners")
public class LimelightFlywheelDistanceTuner extends OpMode {
    private static final int BLUE_TAG_PIPELINE = 0;
    private static final int RED_TAG_PIPELINE = 1;
    private static final int LIMELIGHT_APRILTAG_PIPELINE = BLUE_TAG_PIPELINE;
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;
    private static final double LEFT_INTAKE_DEFAULT_VELOCITY = 900.0;

    private final double[] stepSizes = {1000.0, 500.0, 100.0, 10.0, 1.0};
    private int stepIndex = 2;
    private final double[] intakeStepSizes = {500.0, 100.0, 10.0, 1.0};
    private int intakeStepIndex = 1;

    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;
    private DcMotorEx leftIntake;
    private functions.MegaTag2Prep megaTag2;

    private int goalTagId = BLUE_GOAL_TAG_ID;
    private boolean spinEnabled = false;
    private double manualVelocity = 1800.0;
    private boolean leftIntakeEnabled = false;
    private double leftIntakeVelocity = LEFT_INTAKE_DEFAULT_VELOCITY;

    private Double lastValidDistanceInches = null;
    private Double lastValidTxDegrees = null;

    @Override
    public void init() {
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");
        leftIntake = hardwareMap.get(DcMotorEx.class, "leftintake");

        rightFlyWheel.setDirection(DcMotorEx.Direction.FORWARD);
        leftFlyWheel.setDirection(DcMotorEx.Direction.REVERSE);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        functions.applyFlywheelPidf(leftFlyWheel, rightFlyWheel);
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftIntake.setDirection(DcMotorEx.Direction.FORWARD);
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        functions.applyIntakePidf(leftIntake);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        megaTag2 = new functions.MegaTag2Prep(hardwareMap, telemetry, "limelight", "imu", LIMELIGHT_APRILTAG_PIPELINE);
        megaTag2.initializeAndConfigure();

        telemetry.addLine("LimelightFlywheelDistanceTuner ready");
        telemetry.addLine("Y=flywheel, B=fly step, Dpad L/R=fly vel, A=tag");
        telemetry.addLine("LB=intake, RB=intake step, Dpad U/D=intake vel");
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
        if (gamepad1.leftBumperWasPressed()) leftIntakeEnabled = !leftIntakeEnabled;
        if (gamepad1.rightBumperWasPressed()) intakeStepIndex = (intakeStepIndex + 1) % intakeStepSizes.length;
        if (gamepad1.dpadUpWasPressed()) leftIntakeVelocity += intakeStepSizes[intakeStepIndex];
        if (gamepad1.dpadDownWasPressed()) leftIntakeVelocity = Math.max(0.0, leftIntakeVelocity - intakeStepSizes[intakeStepIndex]);
        if (gamepad1.aWasPressed()) {
            goalTagId = (goalTagId == BLUE_GOAL_TAG_ID) ? RED_GOAL_TAG_ID : BLUE_GOAL_TAG_ID;
            if (megaTag2 != null) {
                megaTag2.switchPipeline(getPipelineForGoalTag(goalTagId));
            }
        }

        Double distanceInches = megaTag2 != null ? megaTag2.getGoalTagHorizontalDistanceInches(goalTagId) : null;
        Double txDegrees = megaTag2 != null ? megaTag2.getGoalTagTxDegrees(goalTagId) : null;
        if (distanceInches != null) lastValidDistanceInches = distanceInches;
        if (txDegrees != null) lastValidTxDegrees = txDegrees;

        double commandVelocity = manualVelocity;

        if (spinEnabled) {
            leftFlyWheel.setVelocity(commandVelocity);
            rightFlyWheel.setVelocity(commandVelocity);
        } else {
            leftFlyWheel.setVelocity(0.0);
            rightFlyWheel.setVelocity(0.0);
        }

        if (leftIntakeEnabled) {
            leftIntake.setVelocity(leftIntakeVelocity);
        } else {
            leftIntake.setVelocity(0.0);
        }

        double leftVelocity = leftFlyWheel.getVelocity();
        double rightVelocity = rightFlyWheel.getVelocity();
        double totalCurrent = leftFlyWheel.getCurrent(CurrentUnit.AMPS) + rightFlyWheel.getCurrent(CurrentUnit.AMPS);
        double leftIntakeMeasuredVelocity = leftIntake.getVelocity();

        telemetry.addLine("----- Limelight + Flywheel Distance Tuner -----");
        telemetry.addData("Tag", goalTagId);
        telemetry.addData("Pipeline", "%d (exp) / %d (active)",
                getPipelineForGoalTag(goalTagId),
                megaTag2 != null ? megaTag2.getActivePipelineIndex() : -1);
        telemetry.addData("Vision", megaTag2 != null ? megaTag2.getPipelineValidationMessage() : "n/a");
        telemetry.addData("Distance in", distanceInches == null ? "n/a" : String.format("%.2f", distanceInches));
        telemetry.addData("Last Dist in", lastValidDistanceInches == null ? "n/a" : String.format("%.2f", lastValidDistanceInches));

        telemetry.addData("Spin", spinEnabled);
        telemetry.addData("Step", "%.1f", stepSizes[stepIndex]);
        telemetry.addData("Set Vel", "%.1f", manualVelocity);
        telemetry.addData("L/R Vel", "%.1f / %.1f", leftVelocity, rightVelocity);
        telemetry.addData("Current A", "%.2f", totalCurrent);
        telemetry.addData("Left Intake", leftIntakeEnabled);
        telemetry.addData("Intake Step", "%.1f", intakeStepSizes[intakeStepIndex]);
        telemetry.addData("Intake Set Vel", "%.1f", leftIntakeVelocity);
        telemetry.addData("Intake Meas Vel", "%.1f", leftIntakeMeasuredVelocity);

        Double distanceForTemplate = distanceInches != null ? distanceInches : lastValidDistanceInches;
        if (distanceForTemplate != null) {
            telemetry.addLine("ShooterData template (paste after tuning this shot):");
            telemetry.addData("Template", "data.put(%.1f, %.0f);", distanceForTemplate, commandVelocity);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        leftFlyWheel.setVelocity(0.0);
        rightFlyWheel.setVelocity(0.0);
        if (leftIntake != null) leftIntake.setVelocity(0.0);
    }

    private int getPipelineForGoalTag(int tagId) {
        return tagId == BLUE_GOAL_TAG_ID ? BLUE_TAG_PIPELINE : RED_TAG_PIPELINE;
    }
}
