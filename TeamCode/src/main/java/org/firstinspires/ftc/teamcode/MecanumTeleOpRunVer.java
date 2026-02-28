package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.shooter.ShooterData;

public abstract class MecanumTeleOpRunVer extends OpMode {
    private static final double DEFAULT_FLYWHEEL_TARGET_VELOCITY = 6000.0;
    private static final double RIGHT_INTAKE_FEED_VELOCITY = 1150.0;
    private static final double RIGHT_INTAKE_REVERSE_VELOCITY = -300.0;
    private static final int BLUE_TAG_PIPELINE = 0;
    private static final int RED_TAG_PIPELINE = 1;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;
    private FaultTolerantMecanumDrive driveController;

    private DcMotorEx rightintake;
    private DcMotorEx leftintake;
    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;

    public double manualFlywheelTargetVelocity = DEFAULT_FLYWHEEL_TARGET_VELOCITY;
    public boolean useVisionFlywheelTarget = false;

    private double activeFlywheelTargetVelocity = DEFAULT_FLYWHEEL_TARGET_VELOCITY;
    private boolean stopping = false;
    private boolean overrideMode = false;

    private Double limelightShooterDistanceInches = null;

    private final ShooterData shooterData = ShooterData.defaultTuned();
    private final functions.TimedIntakePulse feedPulse = new functions.TimedIntakePulse(
            RIGHT_INTAKE_FEED_VELOCITY,
            0.8
    );

    private functions.PinpointIO pinpoint;
    private functions.MegaTag2Prep megaTag2;

    protected abstract String getAllianceName();

    protected abstract int getGoalTagId();

    @Override
    public void init() {
        bindHardware();
        configureHardware();
        initializeLocalizationAndVisionPrep();

        telemetry.addLine("Initialized. Press PLAY.");
        telemetry.update();
    }

    @Override
    public void start() {
        if (driveController != null) driveController.resetForStart();
        if (megaTag2 != null) megaTag2.start();
    }

    @Override
    public void loop() {
        if (stopping) return;

        updateSensors();
        updateOperatorModes();
        updateShooterTargetVelocity();
        updateShotSequence();
        updateIntakeCommands();
        updateFlywheelCommand();
        updateDrive();
        updateTelemetry();
    }

    @Override
    public void stop() {
        stopping = true;

        FaultTolerantMecanumDrive.MotorSafety.setVelocity(leftFlyWheel, 0);
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightFlyWheel, 0);
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightintake, 0);

        FaultTolerantMecanumDrive.MotorSafety.setPower(rightintake, 0);
        FaultTolerantMecanumDrive.MotorSafety.setPower(leftintake, 0);

        if (driveController != null) driveController.stopSafely();
    }

    private void bindHardware() {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        rightintake = hardwareMap.get(DcMotorEx.class, "rightintake");
        leftintake = hardwareMap.get(DcMotorEx.class, "leftintake");
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");
    }

    private void configureHardware() {
        functions.setupMotors(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive,
                leftFlyWheel,
                rightFlyWheel,
                rightintake,
                leftintake
        );

        driveController = new FaultTolerantMecanumDrive(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive,
                telemetry
        );
    }

    private void initializeLocalizationAndVisionPrep() {
        try {
            pinpoint = new functions.PinpointIO(hardwareMap, telemetry, "odo");
            pinpoint.initializeAndConfigure();
        } catch (Exception e) {
            pinpoint = null;
            telemetry.addData("Pinpoint", "Unavailable: %s", e.getClass().getSimpleName());
        }

        megaTag2 = new functions.MegaTag2Prep(
                hardwareMap,
                telemetry,
                "limelight",
                "imu",
                getPipelineForGoalTag(getGoalTagId())
        );
        megaTag2.initializeAndConfigure();
    }

    private void updateSensors() {
        if (pinpoint != null) pinpoint.update();
        if (megaTag2 != null) megaTag2.update();
    }

    private void updateOperatorModes() {
        overrideMode = gamepad2.a;

        // Reuses the old flywheel Y-button slot: now toggles future vision-based targeting.
        if (gamepad1.yWasPressed()) {
            useVisionFlywheelTarget = !useVisionFlywheelTarget;
        }
    }

    private void updateShooterTargetVelocity() {
        limelightShooterDistanceInches = (megaTag2 != null) ? megaTag2.getSuggestedShooterDistanceInches() : null;
        manualFlywheelTargetVelocity = Math.max(0.0, manualFlywheelTargetVelocity);
        activeFlywheelTargetVelocity = functions.computeShooterVelocityTarget(
                shooterData,
                useVisionFlywheelTarget ? limelightShooterDistanceInches : null,
                manualFlywheelTargetVelocity
        );
    }

    private void updateShotSequence() {
        if (gamepad2.bWasPressed()) {
            feedPulse.trigger(rightintake);
        }
        feedPulse.update(rightintake);
    }

    private void updateIntakeCommands() {
        if (feedPulse.isActive()) {
            leftintake.setPower(0.0);
            return;
        }

        if (overrideMode && gamepad2.right_trigger > 0.1) {
            rightintake.setVelocity(RIGHT_INTAKE_REVERSE_VELOCITY);
            leftintake.setPower(0.0);
            return;
        }

        double intakePower = 0.0;
        if (gamepad2.left_trigger > 0.1) {
            intakePower = overrideMode ? -1.0 : 1.0;
        }

        rightintake.setPower(intakePower);
        leftintake.setPower(intakePower);
    }

    private void updateFlywheelCommand() {
        if (gamepad2.right_bumper) {
            functions.setTwoMotorsVelocity(leftFlyWheel, rightFlyWheel, activeFlywheelTargetVelocity);
        } else {
            functions.setTwoMotorsVelocity(leftFlyWheel, rightFlyWheel, 0.0);
        }
    }

    private void updateDrive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        if (driveController != null) driveController.move(drive, strafe, turn);
    }

    private void updateTelemetry() {
        telemetry.addLine("--------------------------------------");
        telemetry.addData("Alliance", getAllianceName());
        telemetry.addData("Goal Tag ID", getGoalTagId());
        telemetry.addData("Override Mode", overrideMode);
        telemetry.addData("Vision Flywheel Target", useVisionFlywheelTarget);
        telemetry.addData("Shooter Distance (in)", limelightShooterDistanceInches == null ? "n/a" : String.format("%.1f", limelightShooterDistanceInches));
        telemetry.addData("Flywheel Target Velocity", "%.1f", activeFlywheelTargetVelocity);
        telemetry.addData("Flywheel Target Source", useVisionFlywheelTarget ? "Vision/ShooterData" : "Manual");
        telemetry.addData("Flywheel PIDF", "P=%.1f I=%.1f D=%.1f F=%.1f",
                functions.FLYWHEEL_P, functions.FLYWHEEL_I, functions.FLYWHEEL_D, functions.FLYWHEEL_F);
        telemetry.addData("Left Wheel Velocity", "%.1f", leftFlyWheel.getVelocity());
        telemetry.addData("Right Wheel Velocity", "%.1f", rightFlyWheel.getVelocity());
        telemetry.addData("Feed Pulse Active", feedPulse.isActive());
        if (feedPulse.isActive()) {
            telemetry.addData("Feed Pulse t", "%.2f", feedPulse.getElapsedSeconds());
        }

        if (pinpoint != null) {
            telemetry.addData("Odo X (in)", "%.1f", pinpoint.getXInches());
            telemetry.addData("Odo Y (in)", "%.1f", pinpoint.getYInches());
            telemetry.addData("Odo H (deg)", "%.1f", Math.toDegrees(pinpoint.getHeadingRadians()));
        } else {
            telemetry.addData("Odo", "Unavailable");
        }

        if (megaTag2 != null) {
            megaTag2.addTelemetry(telemetry);
        }

        telemetry.update();
    }

    private int getPipelineForGoalTag(int goalTagId) {
        return goalTagId == 20 ? BLUE_TAG_PIPELINE : RED_TAG_PIPELINE;
    }
}
