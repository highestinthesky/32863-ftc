package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.shooter.AutoShotSequenceController;
import org.firstinspires.ftc.teamcode.shooter.ShooterData;
import org.firstinspires.ftc.teamcode.shooter.ShotControlConfig;
import org.firstinspires.ftc.teamcode.shooter.TurretCrServoController;

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
    private AutoShotSequenceController autoShotController;
    private double commandedLeftIntakePower = Double.NaN;
    private double commandedRightIntakePower = Double.NaN;
    private boolean goalTagDetected = false;
    private boolean faultModeEnabled = false;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        bindHardware();
        configureHardware();
        initializeLocalizationAndVisionPrep();

        telemetry.addLine("Initialized. Press PLAY.");
        telemetry.update();
    }

    @Override
    public void start() {
        stopping = false;
        commandedLeftIntakePower = Double.NaN;
        commandedRightIntakePower = Double.NaN;
        if (driveController != null) driveController.resetForStart();
        if (driveController != null) driveController.setFaultModeEnabled(faultModeEnabled);
        if (megaTag2 != null) megaTag2.start();
    }

    @Override
    public void loop() {
        if (stopping) return;

        updateSensors();
        updateAutoShotSequence();
        updateIntakeCommands();
        updateDriveFaultModeToggle();
        updateDrive();
        updateTelemetry();
    }

    @Override
    public void stop() {
        stopping = true;

        if (autoShotController != null) {
            autoShotController.stopAll();
        } else {
            FaultTolerantMecanumDrive.MotorSafety.setVelocity(leftFlyWheel, 0);
            FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightFlyWheel, 0);
            FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightintake, 0);
            FaultTolerantMecanumDrive.MotorSafety.setVelocity(leftintake, 0);
        }
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
        autoShotController = new AutoShotSequenceController(
                leftFlyWheel,
                rightFlyWheel,
                rightintake,
                leftintake,
                megaTag2,
                shooterData,
                GOAL_TAG_ID,
                turretController,
                shotConfig
        );
    }

    private void updateSensors() {
        if (megaTag2 != null) {
            megaTag2.update();
            goalTagDetected = megaTag2.hasGoalTag(GOAL_TAG_ID);
        } else {
            goalTagDetected = false;
        }
    }

    private void updateAutoShotSequence() {
        if (autoShotController != null) {
            autoShotController.update(gamepad2.bWasPressed(), false);
        }
    }

    private void updateIntakeCommands() {
        double leftPower = computeIntakePower(gamepad2.left_trigger, gamepad2.left_bumper);
        double rightPower = computeIntakePower(gamepad2.right_trigger, gamepad2.right_bumper);
        setIntakePowers(leftPower, rightPower);
    }

    private void updateDrive() {
        double drive = applyDeadband(-gamepad1.left_stick_y, DRIVE_STICK_DEADBAND);
        double strafe = applyDeadband(-gamepad1.left_stick_x, DRIVE_STICK_DEADBAND);
        double turn = applyDeadband(-gamepad1.right_stick_x, DRIVE_STICK_DEADBAND);

        if (driveController != null) driveController.move(drive, strafe, turn);
    }

    private void updateTelemetry() {
        telemetry.addLine("--------------------------------------");
        telemetry.addData("Alliance", "Blue");
        telemetry.addData("Goal Tag ID", GOAL_TAG_ID);
        telemetry.addData("Shoot Buttons", "B=shoot, X=disabled");
        telemetry.addData("Fault Mode", "%s (Y toggles)", faultModeEnabled ? "ON" : "OFF");
        telemetry.addData("Intake Controls", "LT/LB=left in/out, RT/RB=right in/out");
        telemetry.addData("AprilTag Detected", goalTagDetected);
        if (autoShotController != null && autoShotController.isFlywheelChargedUp()) {
            telemetry.addLine("Flywheel charged up");
        }

        if (megaTag2 != null) {
            megaTag2.addTelemetry(telemetry);
        }
        if (autoShotController != null) {
            autoShotController.addTelemetry(telemetry);
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

    private void updateDriveFaultModeToggle() {
        if (!gamepad1.yWasPressed()) return;
        faultModeEnabled = !faultModeEnabled;
        if (driveController != null) driveController.setFaultModeEnabled(faultModeEnabled);
    }
}
