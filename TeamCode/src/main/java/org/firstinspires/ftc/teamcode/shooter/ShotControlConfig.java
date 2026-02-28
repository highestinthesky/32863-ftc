package org.firstinspires.ftc.teamcode.shooter;

/**
 * Central location for one-button auto-shot tuning values.
 * <p>
 * TODO (HUGE): tune every value below on-robot.
 * - Flywheel idle/target readiness thresholds
 * - 2-shot and 3-shot feed timings/velocities
 * - Turret aiming deadband/P gain/max power
 * - Turret return-home behavior
 * - Intake split behavior for front vs transfer wheels
 * - Consecutive-shot compensation thresholds and gains
 */
public class ShotControlConfig {
    public double flywheelIdleVelocity = 2000.0;
    public double flywheelReadyTolerance = 180.0;
    public double flywheelReadySettleSeconds = 0.15;
    public double flywheelSpinupTimeoutSeconds = 1.25;

    // Turret auto-aim tune values (CR servo, no absolute position sensor).
    public double turretAimKp = 0.04;
    public double turretAimDeadbandDegrees = 1.0;
    public double turretAimMaxPower = 0.30;
    public double turretAimDirection = 1.0;
    public double turretAimTimeoutSeconds = 0.90;

    public double turretReturnPower = 0.25;
    public double turretReturnStopThreshold = 0.02;
    public double turretReturnTimeoutSeconds = 1.20;

    // 2-ball sequence: transfer-only.
    public double twoShotTransferVelocity = 6000.0;
    public double twoShotTransferDurationSeconds = 0.70;

    // 3-ball sequence: transfer backoff then both forward.
    public double threeShotTransferBackoffVelocity = -200.0;
    public double threeShotTransferBackoffDurationSeconds = 0.18;
    public double threeShotFrontForwardVelocity = 1200.0;
    public double threeShotTransferForwardVelocity = 950.0;
    public double threeShotForwardDurationSeconds = 1.25;

    // Manual collection controls (when auto-shot is idle).
    public double intakeCollectPower = 1.0;
    public double intakeReversePower = -1.0;
    public double intakeReverseVelocity = -300.0;
    public double triggerDeadband = 0.1;

    // Disabled-by-default compensation scaffold.
    public boolean enableConsecutiveCompensation = false;
    public double compensationVelocityDropThreshold = 160.0;
    public double compensationBoostVelocity = 120.0;
    public double compensationReleaseHysteresis = 60.0;

    public static ShotControlConfig defaultConfig() {
        return new ShotControlConfig();
    }
}
