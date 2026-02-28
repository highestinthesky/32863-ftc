package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.functions;

public class TurretCrServoController {
    public enum AimResult {
        AIMING,
        CENTERED,
        TIMEOUT,
        UNAVAILABLE
    }

    private CRServo rightTurret;
    private CRServo leftTurret;
    private boolean available;
    private String status = "Uninitialized";

    private final ElapsedTime aimAttemptTimer = new ElapsedTime();
    private boolean aimAttemptActive = false;

    /**
     * Signed estimate of turn offset from "home" based on commanded power*time.
     * Positive and negative commands naturally cancel.
     */
    private double estimatedTurnOffset = 0.0;
    private double lastAppliedPower = 0.0;
    private boolean returningHome = false;

    public TurretCrServoController(HardwareMap hardwareMap, String rightServoName, String leftServoName) {
        try {
            rightTurret = hardwareMap.get(CRServo.class, rightServoName);
            leftTurret = hardwareMap.get(CRServo.class, leftServoName);

            rightTurret.setDirection(DcMotorSimple.Direction.FORWARD);
            leftTurret.setDirection(DcMotorSimple.Direction.FORWARD);
            setPower(0.0);

            available = true;
            status = "Ready";
        } catch (Exception e) {
            available = false;
            status = "Unavailable: " + e.getClass().getSimpleName();
        }
    }

    public boolean isAvailable() {
        return available;
    }

    public String getStatus() {
        return status;
    }

    public double getEstimatedTurnOffset() {
        return estimatedTurnOffset;
    }

    public void beginAimAttempt() {
        aimAttemptTimer.reset();
        aimAttemptActive = true;
        returningHome = false;
        lastAppliedPower = 0.0;
        status = "Aiming";
    }

    public AimResult aimToTx(double txDegrees, double dtSeconds, ShotControlConfig config) {
        return aimToTx(txDegrees, dtSeconds, config, true);
    }

    public AimResult aimToTx(double txDegrees, double dtSeconds, ShotControlConfig config, boolean enforceTimeout) {
        if (!available) return AimResult.UNAVAILABLE;
        if (!aimAttemptActive) beginAimAttempt();

        if (enforceTimeout && aimAttemptTimer.seconds() > config.turretAimTimeoutSeconds) {
            applyTurnCommand(0.0, dtSeconds, false);
            status = "Aim timeout";
            return AimResult.TIMEOUT;
        }

        if (Math.abs(txDegrees) <= config.turretAimDeadbandDegrees) {
            applyTurnCommand(0.0, dtSeconds, false);
            status = "Centered";
            return AimResult.CENTERED;
        }

        double targetCommand = txDegrees * config.turretAimKp * config.turretAimDirection;
        targetCommand = functions.clamp(targetCommand, -Math.abs(config.turretAimMaxPower), Math.abs(config.turretAimMaxPower));
        double command = rateLimitCommand(targetCommand, dtSeconds, config);
        applyTurnCommand(command, dtSeconds, true);
        status = String.format("Aiming tx=%.2f pwr=%.2f", txDegrees, command);
        return AimResult.AIMING;
    }

    public void startReturnHome() {
        if (!available) return;
        returningHome = true;
        aimAttemptActive = false;
        lastAppliedPower = 0.0;
        status = "Returning home";
    }

    public boolean updateReturnHome(double dtSeconds, ShotControlConfig config) {
        if (!available) return true;
        if (!returningHome) return true;

        if (Math.abs(estimatedTurnOffset) <= config.turretReturnStopThreshold) {
            estimatedTurnOffset = 0.0;
            applyTurnCommand(0.0, dtSeconds, false);
            returningHome = false;
            status = "Home";
            return true;
        }

        double previousOffset = estimatedTurnOffset;
        double returnCommand = -Math.signum(estimatedTurnOffset) * Math.abs(config.turretReturnPower);
        applyTurnCommand(returnCommand, dtSeconds, true);

        // Clamp small overshoot to zero.
        if (Math.signum(previousOffset) != Math.signum(estimatedTurnOffset)) {
            estimatedTurnOffset = 0.0;
            applyTurnCommand(0.0, dtSeconds, false);
            returningHome = false;
            status = "Home";
            return true;
        }

        status = String.format("Returning offset=%.3f", estimatedTurnOffset);
        return false;
    }

    public void stop() {
        aimAttemptActive = false;
        returningHome = false;
        if (available) setPower(0.0);
        status = available ? "Stopped" : status;
    }

    private void applyTurnCommand(double power, double dtSeconds, boolean trackOffset) {
        setPower(power);
        if (trackOffset && dtSeconds > 0.0) {
            estimatedTurnOffset += power * dtSeconds;
        }
    }

    private double rateLimitCommand(double targetCommand, double dtSeconds, ShotControlConfig config) {
        if (config == null) return targetCommand;
        double safeDt = dtSeconds > 0.0 ? dtSeconds : 0.02;
        double maxDelta = Math.abs(config.turretAimSlewRatePerSecond) * safeDt;
        double lower = lastAppliedPower - maxDelta;
        double upper = lastAppliedPower + maxDelta;
        return functions.clamp(targetCommand, lower, upper);
    }

    private void setPower(double power) {
        if (!available) return;
        lastAppliedPower = power;
        leftTurret.setPower(power);
        rightTurret.setPower(power);
    }

    public double getLastAppliedPower() {
        return lastAppliedPower;
    }
}
