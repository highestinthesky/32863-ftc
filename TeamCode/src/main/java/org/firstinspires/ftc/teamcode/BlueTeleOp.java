package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shooter.ShooterData;
import org.firstinspires.ftc.teamcode.shooter.ShotControlConfig;
import org.firstinspires.ftc.teamcode.shooter.TurretCrServoController;
import org.firstinspires.ftc.teamcode.shooter.VisionShooterTargeting;

@TeleOp(name = "BlueTeleOp")
public class BlueTeleOp extends OpMode {
    private static final int GOAL_TAG_ID = 20;
    private static final int LIMELIGHT_APRILTAG_PIPELINE = 0;
    private static final String TURRET_RIGHT_SERVO = "rightturretturn";
    private static final String TURRET_LEFT_SERVO = "leftturretturn";
    private static final double INTAKE_IN_POWER = 1.0;
    private static final double INTAKE_OUT_SLOW_POWER = -1.0 / 3.0;
    private static final double DRIVE_STICK_DEADBAND = 0.05;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;
    private FaultTolerantMecanumDrive driveController;

    private DcMotorEx rightintake;
    private DcMotorEx leftintake;
    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;

    private boolean stopping = false;
    private final ShotControlConfig shotConfig = ShotControlConfig.defaultConfig();
    private final ShooterData shooterData = ShooterData.defaultTuned();

    private functions.MegaTag2Prep megaTag2;
    private TurretCrServoController turretController;
    private VisionShooterTargeting visionTargeting;
    private double commandedLeftIntakePower = Double.NaN;
    private double commandedRightIntakePower = Double.NaN;
    private double commandedFlywheelVelocity = 0.0;
    private double visionTargetVelocity = 0.0;
    private Double lastSeenTxDegrees = null;
    private boolean visionFeedModeActive = false;
    private String visionFeedStatus = "Idle";
    private boolean goalTagDetected = false;
    private boolean faultModeEnabled = false;
    private Servo turrethood;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopSeconds = 0.0;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        bindHardware();
        configureHardware();
        initializeLocalizationAndVisionPrep();

         turrethood = hardwareMap.get(Servo.class, "tservo");
         turrethood.setDirection(Servo.Direction.REVERSE);

         telemetry.addLine("Initialized. Press PLAY.");
         telemetry.update();
    }

    @Override
    public void start() {
        stopping = false;
        commandedLeftIntakePower = Double.NaN;
        commandedRightIntakePower = Double.NaN;
        commandedFlywheelVelocity = shotConfig.flywheelIdleVelocity;
        visionTargetVelocity = 0.0;
        lastSeenTxDegrees = null;
        visionFeedModeActive = false;
        visionFeedStatus = "Idle";
        lastLoopSeconds = 0.0;
        if (driveController != null) driveController.resetForStart();
        if (driveController != null) driveController.setFaultModeEnabled(faultModeEnabled);
        if (megaTag2 != null) megaTag2.start();
        setFlywheelVelocity(shotConfig.flywheelIdleVelocity);
        if (turretController != null) turretController.stop();

        turrethood.setPosition(1);
    }

    @Override
    public void loop() {
        if (stopping) return;

        updateSensors();
        updateVisionFeedMode();
        updateIntakeCommands();
        updateDriveFaultModeToggle();
        updateDrive();
        updateTelemetry();
    }

    @Override
    public void stop() {
        turrethood.setPosition(0);
        stopping = true;

        setFlywheelVelocity(0.0);
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightintake, 0);
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(leftintake, 0);
        setIntakePowers(0.0, 0.0);

        if (turretController != null) turretController.stop();

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
        leftFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftintake.setDirection(DcMotorSimple.Direction.REVERSE);

        driveController = new FaultTolerantMecanumDrive(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive,
                telemetry
        );
    }

    private void initializeLocalizationAndVisionPrep() {
        megaTag2 = new functions.MegaTag2Prep(hardwareMap, telemetry, "limelight", "imu", LIMELIGHT_APRILTAG_PIPELINE);
        megaTag2.initializeAndConfigure();

        turretController = new TurretCrServoController(hardwareMap, TURRET_RIGHT_SERVO, TURRET_LEFT_SERVO);
        visionTargeting = new VisionShooterTargeting(megaTag2, shooterData, GOAL_TAG_ID);
    }

    private void updateSensors() {
        if (megaTag2 != null) {
            megaTag2.update();
            goalTagDetected = megaTag2.hasGoalTag(GOAL_TAG_ID);
        } else {
            goalTagDetected = false;
        }
    }

    private void updateVisionFeedMode() {
        if (gamepad2.bWasPressed() && !visionFeedModeActive) {
            visionFeedModeActive = true;
            visionFeedStatus = "Vision feed active";
            visionTargetVelocity = 0.0;
            if (turretController != null) turretController.beginAimAttempt();
        }

        if (gamepad2.aWasPressed() && visionFeedModeActive) {
            stopVisionFeedMode();
            return;
        }

        if (!visionFeedModeActive) {
            setFlywheelVelocity(shotConfig.flywheelIdleVelocity);
            return;
        }

        boolean hasTarget = false;
        Double aimTx = lastSeenTxDegrees;
        if (visionTargeting != null) {
            visionTargeting.update();
            hasTarget = visionTargeting.isTargetAvailable();
            if (hasTarget) {
                aimTx = visionTargeting.getTagTxDegrees();
                if (aimTx != null) lastSeenTxDegrees = aimTx;
                visionTargetVelocity = Math.max(0.0, visionTargeting.getTargetVelocity());
            }
        }

        if (aimTx == null) {
            aimTx = shotConfig.turretAimCoarseTxThresholdDegrees;
            lastSeenTxDegrees = aimTx;
        }

        if (turretController != null && turretController.isAvailable()) {
            turretController.aimToTx(aimTx, computeDtSeconds(), shotConfig, false);
        }

        double activeVelocity = visionTargetVelocity > 0.0 ? visionTargetVelocity : shotConfig.flywheelIdleVelocity;
        setFlywheelVelocity(activeVelocity);

        // During active vision-feed mode, human intake controls are intentionally ignored.
        setIntakePowers(INTAKE_IN_POWER, INTAKE_IN_POWER);
        visionFeedStatus = hasTarget ? "Tracking and feeding" : "Searching using last tx";
    }

    private void updateIntakeCommands() {
        if (visionFeedModeActive) return;
        double leftPower = computeIntakePower(gamepad2.left_trigger, gamepad2.left_bumper);
        double rightPower = computeIntakePower(gamepad2.right_trigger, gamepad2.right_bumper);
        setIntakePowers(leftPower, rightPower);
    }

    private void updateDrive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        if (driveController != null) driveController.move(drive, strafe, turn);
    }

    private void updateTelemetry() {
        telemetry.addLine("--------------------------------------");
        telemetry.addData("Alliance", "Blue");
        telemetry.addData("Goal Tag ID", GOAL_TAG_ID);
        telemetry.addData("Shoot Buttons", "B=start vision feed, A=stop");
        telemetry.addData("Fault Mode", "%s (Y toggles)", faultModeEnabled ? "ON" : "OFF");
        telemetry.addData("Intake Controls", "LT/LB=left in/out, RT/RB=right in/out");
        telemetry.addData("AprilTag Detected", goalTagDetected);
        telemetry.addData("Vision Feed Mode", visionFeedModeActive ? "ACTIVE" : "IDLE");
        telemetry.addData("Vision Feed Status", visionFeedStatus);
        telemetry.addData("Flywheel Cmd Vel", "%.0f", commandedFlywheelVelocity);
        if (isFlywheelChargedUp()) {
            telemetry.addLine("Flywheel charged up");
        }

        if (megaTag2 != null) {
            megaTag2.addTelemetry(telemetry);
        }
        if (visionTargeting != null) {
            telemetry.addData("Vision", visionTargeting.getStatus());
        }
        if (turretController != null) {
            telemetry.addData("Turret Status", turretController.getStatus());
            telemetry.addData("Turret Cmd Pwr", "%.2f", turretController.getLastAppliedPower());
        }

        telemetry.update();
    }

    private double computeIntakePower(double triggerValue, boolean reverseButtonPressed) {
        if (reverseButtonPressed) return INTAKE_OUT_SLOW_POWER;
        if (triggerValue > shotConfig.triggerDeadband) return INTAKE_IN_POWER;
        return 0.0;
    }

    private void setIntakePowers(double leftPower, double rightPower) {
        if (Double.isNaN(commandedLeftIntakePower) || Math.abs(leftPower - commandedLeftIntakePower) > 1e-4) {
            leftintake.setPower(leftPower);
            commandedLeftIntakePower = leftPower;
        }
        if (Double.isNaN(commandedRightIntakePower) || Math.abs(rightPower - commandedRightIntakePower) > 1e-4) {
            rightintake.setPower(rightPower);
            commandedRightIntakePower = rightPower;
        }
    }

    private static double applyDeadband(double value, double deadband) {
        return Math.abs(value) >= Math.abs(deadband) ? value : 0.0;
    }

    private void setFlywheelVelocity(double velocity) {
        commandedFlywheelVelocity = velocity;
        if (leftFlyWheel != null) leftFlyWheel.setVelocity(velocity);
        if (rightFlyWheel != null) rightFlyWheel.setVelocity(velocity);
    }

    private double computeDtSeconds() {
        double now = loopTimer.seconds();
        double dt = now - lastLoopSeconds;
        lastLoopSeconds = now;
        if (dt <= 0.0) return 0.02;
        return Math.min(dt, 0.1);
    }

    private void stopVisionFeedMode() {
        visionFeedModeActive = false;
        visionFeedStatus = "Stopped by A";
        visionTargetVelocity = 0.0;
        lastSeenTxDegrees = null;
        setFlywheelVelocity(shotConfig.flywheelIdleVelocity);
        setIntakePowers(0.0, 0.0);
        if (turretController != null) turretController.stop();
    }

    private boolean isFlywheelChargedUp() {
        double target = Math.abs(commandedFlywheelVelocity);
        double idle = Math.abs(shotConfig.flywheelIdleVelocity);
        if (target <= idle + 100.0) return false;
        double leftSpeed = Math.abs(leftFlyWheel.getVelocity());
        double rightSpeed = Math.abs(rightFlyWheel.getVelocity());
        return Math.abs(leftSpeed - target) <= shotConfig.flywheelReadyTolerance
                && Math.abs(rightSpeed - target) <= shotConfig.flywheelReadyTolerance;
    }

    private void updateDriveFaultModeToggle() {
        if (!gamepad1.yWasPressed()) return;
        faultModeEnabled = !faultModeEnabled;
        if (driveController != null) driveController.setFaultModeEnabled(faultModeEnabled);
    }
}
