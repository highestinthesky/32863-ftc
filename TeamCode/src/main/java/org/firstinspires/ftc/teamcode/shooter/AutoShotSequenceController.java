package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.functions;

public class AutoShotSequenceController {
    public enum SequenceType {
        NONE,
        TWO_BALL,
        THREE_BALL
    }

    public enum State {
        IDLE,
        ACQUIRE_TAG,
        AIM,
        SPINUP,
        FEED_TWO,
        FEED_THREE_BACKOFF,
        FEED_THREE_FORWARD,
        RETURN_HOME,
        COMPLETE,
        ABORT
    }

    private final DcMotorEx leftFlyWheel;
    private final DcMotorEx rightFlyWheel;
    private final DcMotorEx rightIntakeFront;
    private final DcMotorEx leftIntakeTransfer;

    private final ShotControlConfig config;
    private final VisionShooterTargeting targeting;
    private final TurretCrServoController turret;
    private final ConsecutiveShotCompensator compensator;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private double lastLoopSeconds = 0.0;
    private boolean readyWindowActive = false;

    private State state = State.IDLE;
    private SequenceType activeSequence = SequenceType.NONE;
    private String status = "Idle";
    private String abortReason = "";

    private Double targetDistanceInches = null;
    private Double targetTxDegrees = null;
    private double targetVelocity = 0.0;
    private double commandedFlywheelVelocity = 0.0;

    public AutoShotSequenceController(
            DcMotorEx leftFlyWheel,
            DcMotorEx rightFlyWheel,
            DcMotorEx rightIntakeFront,
            DcMotorEx leftIntakeTransfer,
            functions.MegaTag2Prep megaTag2,
            ShooterData shooterData,
            int goalTagId,
            TurretCrServoController turret,
            ShotControlConfig config
    ) {
        this.leftFlyWheel = leftFlyWheel;
        this.rightFlyWheel = rightFlyWheel;
        this.rightIntakeFront = rightIntakeFront;
        this.leftIntakeTransfer = leftIntakeTransfer;
        this.turret = turret;
        this.config = config != null ? config : ShotControlConfig.defaultConfig();
        this.targeting = new VisionShooterTargeting(megaTag2, shooterData, goalTagId);
        this.compensator = new ConsecutiveShotCompensator(this.config);
    }

    public void update(boolean twoBallPressed, boolean threeBallPressed) {
        double dt = computeDtSeconds();

        if (state == State.IDLE) {
            setFlywheelVelocity(config.flywheelIdleVelocity);
            if (twoBallPressed) {
                startSequence(SequenceType.TWO_BALL);
            } else if (threeBallPressed) {
                startSequence(SequenceType.THREE_BALL);
            }
            return;
        }

        switch (state) {
            case ACQUIRE_TAG:
                handleAcquireTag();
                break;
            case AIM:
                handleAim(dt);
                break;
            case SPINUP:
                handleSpinup();
                break;
            case FEED_TWO:
                handleFeedTwo();
                break;
            case FEED_THREE_BACKOFF:
                handleFeedThreeBackoff();
                break;
            case FEED_THREE_FORWARD:
                handleFeedThreeForward();
                break;
            case RETURN_HOME:
                handleReturnHome(dt);
                break;
            case COMPLETE:
                finishSequence();
                break;
            case ABORT:
                handleAbort();
                break;
            case IDLE:
            default:
                break;
        }
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    public void stopAll() {
        state = State.IDLE;
        activeSequence = SequenceType.NONE;
        stopIntakes();
        setFlywheelVelocity(0.0);
        if (turret != null) turret.stop();
        status = "Stopped";
    }

    public State getState() {
        return state;
    }

    public String getStatus() {
        return status;
    }

    public String getAbortReason() {
        return abortReason;
    }

    public SequenceType getActiveSequence() {
        return activeSequence;
    }

    public Double getTargetDistanceInches() {
        return targetDistanceInches;
    }

    public Double getTargetTxDegrees() {
        return targetTxDegrees;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCommandedFlywheelVelocity() {
        return commandedFlywheelVelocity;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (telemetry == null) return;
        telemetry.addData("AutoShot State", state);
        telemetry.addData("AutoShot Sequence", activeSequence);
        telemetry.addData("AutoShot Status", status);
        telemetry.addData("AutoShot Abort", abortReason.isEmpty() ? "none" : abortReason);
        telemetry.addData("AutoShot Distance (in)", targetDistanceInches == null ? "n/a" : String.format("%.1f", targetDistanceInches));
        telemetry.addData("AutoShot tx (deg)", targetTxDegrees == null ? "n/a" : String.format("%.2f", targetTxDegrees));
        telemetry.addData("AutoShot Target Vel", "%.1f", targetVelocity);
        telemetry.addData("AutoShot Cmd Vel", "%.1f", commandedFlywheelVelocity);
        telemetry.addData("Compensator", compensator.getStatus());
        if (turret != null) {
            telemetry.addData("Turret Status", turret.getStatus());
            telemetry.addData("Turret Offset Est", "%.3f", turret.getEstimatedTurnOffset());
            telemetry.addData("Turret Cmd Pwr", "%.2f", turret.getLastAppliedPower());
        }
        telemetry.addData("Vision", targeting.getStatus());
    }

    private void startSequence(SequenceType sequenceType) {
        abortReason = "";
        activeSequence = sequenceType;
        targetDistanceInches = null;
        targetTxDegrees = null;
        targetVelocity = 0.0;
        commandedFlywheelVelocity = config.flywheelIdleVelocity;
        readyWindowActive = false;
        enterState(State.ACQUIRE_TAG);
        status = sequenceType == SequenceType.TWO_BALL ? "2-ball requested" : "3-ball requested";
    }

    private void handleAcquireTag() {
        setFlywheelVelocity(config.flywheelIdleVelocity);
        stopIntakes();

        targeting.update();
        if (!targeting.isTargetAvailable()) {
            abortSequence("No alliance goal tag: " + targeting.getStatus());
            return;
        }

        targetDistanceInches = targeting.getDistanceInches();
        targetTxDegrees = targeting.getTagTxDegrees();
        targetVelocity = Math.max(0.0, targeting.getTargetVelocity());

        if (turret == null || !turret.isAvailable()) {
            abortSequence("Turret unavailable");
            return;
        }

        turret.beginAimAttempt();
        enterState(State.AIM);
        status = "Tag acquired";
    }

    private void handleAim(double dtSeconds) {
        setFlywheelVelocity(config.flywheelIdleVelocity);
        stopIntakes();

        targeting.update();
        if (!targeting.isTargetAvailable()) {
            abortSequence("Lost alliance goal tag while aiming");
            return;
        }

        targetDistanceInches = targeting.getDistanceInches();
        targetTxDegrees = targeting.getTagTxDegrees();
        targetVelocity = Math.max(0.0, targeting.getTargetVelocity());

        Double tx = targetTxDegrees;
        if (tx == null) {
            abortSequence("Tag tx unavailable");
            return;
        }

        TurretCrServoController.AimResult aimResult = turret.aimToTx(tx, dtSeconds, config);
        if (aimResult == TurretCrServoController.AimResult.CENTERED) {
            enterState(State.SPINUP);
            status = "Turret centered";
        } else if (aimResult == TurretCrServoController.AimResult.TIMEOUT) {
            abortSequence("Turret aim timeout");
        } else if (aimResult == TurretCrServoController.AimResult.UNAVAILABLE) {
            abortSequence("Turret unavailable");
        } else {
            status = "Aiming turret";
        }
    }

    private void handleSpinup() {
        stopIntakes();
        commandedFlywheelVelocity = computeCommandedVelocity();
        setFlywheelVelocity(commandedFlywheelVelocity);

        double leftSpeed = Math.abs(leftFlyWheel.getVelocity());
        double rightSpeed = Math.abs(rightFlyWheel.getVelocity());
        double target = Math.abs(commandedFlywheelVelocity);

        boolean ready = Math.abs(leftSpeed - target) <= config.flywheelReadyTolerance
                && Math.abs(rightSpeed - target) <= config.flywheelReadyTolerance;

        if (ready) {
            if (!readyWindowActive) {
                readyWindowActive = true;
                readyTimer.reset();
            }
        } else {
            readyWindowActive = false;
        }

        boolean settleMet = readyWindowActive && readyTimer.seconds() >= config.flywheelReadySettleSeconds;
        boolean timeoutMet = stateTimer.seconds() >= config.flywheelSpinupTimeoutSeconds;
        if (settleMet || timeoutMet) {
            enterState(activeSequence == SequenceType.THREE_BALL ? State.FEED_THREE_BACKOFF : State.FEED_TWO);
            status = settleMet ? "Spinup ready" : "Spinup timeout fallback";
        } else {
            status = "Spinning up";
        }
    }

    private void handleFeedTwo() {
        commandedFlywheelVelocity = computeCommandedVelocity();
        setFlywheelVelocity(commandedFlywheelVelocity);

        setIntakeVelocity(0.0, config.twoShotTransferVelocity);
        if (stateTimer.seconds() >= config.twoShotTransferDurationSeconds) {
            enterState(State.RETURN_HOME);
            status = "2-ball feed complete";
        } else {
            status = "Feeding 2-ball";
        }
    }

    private void handleFeedThreeBackoff() {
        commandedFlywheelVelocity = computeCommandedVelocity();
        setFlywheelVelocity(commandedFlywheelVelocity);

        setIntakeVelocity(0.0, config.threeShotTransferBackoffVelocity);
        if (stateTimer.seconds() >= config.threeShotTransferBackoffDurationSeconds) {
            enterState(State.FEED_THREE_FORWARD);
            status = "3-ball backoff complete";
        } else {
            status = "3-ball transfer backoff";
        }
    }

    private void handleFeedThreeForward() {
        commandedFlywheelVelocity = computeCommandedVelocity();
        setFlywheelVelocity(commandedFlywheelVelocity);

        // TODO (HUGE): tune front vs transfer velocities and timing for reliable 3-ball feed.
        setIntakeVelocity(config.threeShotFrontForwardVelocity, config.threeShotTransferForwardVelocity);
        if (stateTimer.seconds() >= config.threeShotForwardDurationSeconds) {
            enterState(State.RETURN_HOME);
            status = "3-ball feed complete";
        } else {
            status = "Feeding 3-ball";
        }
    }

    private void handleReturnHome(double dtSeconds) {
        stopIntakes();
        setFlywheelVelocity(config.flywheelIdleVelocity);

        if (turret == null || !turret.isAvailable()) {
            enterState(State.COMPLETE);
            status = "No turret return needed";
            return;
        }

        boolean homed = turret.updateReturnHome(dtSeconds, config);
        if (homed || stateTimer.seconds() >= config.turretReturnTimeoutSeconds) {
            enterState(State.COMPLETE);
            status = homed ? "Turret homed" : "Turret return timeout";
        } else {
            status = "Returning turret home";
        }
    }

    private void handleAbort() {
        stopIntakes();
        setFlywheelVelocity(config.flywheelIdleVelocity);
        if (turret != null && turret.isAvailable()) turret.startReturnHome();
        enterState(State.RETURN_HOME);
    }

    private void finishSequence() {
        stopIntakes();
        setFlywheelVelocity(config.flywheelIdleVelocity);
        activeSequence = SequenceType.NONE;
        state = State.IDLE;
        status = abortReason.isEmpty() ? "Idle" : "Idle after abort";
    }

    private void abortSequence(String reason) {
        abortReason = reason;
        status = reason;
        enterState(State.ABORT);
    }

    private void enterState(State newState) {
        state = newState;
        stateTimer.reset();
        if (newState == State.SPINUP) {
            readyWindowActive = false;
        }
        if (newState == State.RETURN_HOME && turret != null && turret.isAvailable()) {
            turret.startReturnHome();
        }
    }

    private double computeCommandedVelocity() {
        return compensator.apply(
                targetVelocity,
                Math.abs(leftFlyWheel.getVelocity()),
                Math.abs(rightFlyWheel.getVelocity())
        );
    }

    private double computeDtSeconds() {
        double current = loopTimer.seconds();
        double dt = current - lastLoopSeconds;
        lastLoopSeconds = current;
        if (dt <= 0.0) return 0.02;
        return Math.min(dt, 0.1);
    }

    private void setFlywheelVelocity(double velocity) {
        commandedFlywheelVelocity = velocity;
        if (leftFlyWheel != null) leftFlyWheel.setVelocity(velocity);
        if (rightFlyWheel != null) rightFlyWheel.setVelocity(velocity);
    }

    private void stopIntakes() {
        setIntakeVelocity(0.0, 0.0);
    }

    private void setIntakeVelocity(double rightFrontVelocity, double leftTransferVelocity) {
        if (rightIntakeFront != null) rightIntakeFront.setVelocity(rightFrontVelocity);
        if (leftIntakeTransfer != null) leftIntakeTransfer.setVelocity(leftTransferVelocity);
    }
}
