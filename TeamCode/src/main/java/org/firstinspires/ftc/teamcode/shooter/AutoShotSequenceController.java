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

    private int lostTagFrameCount = 0;
    private int centeredFrameCount = 0;
    private Double lastSeenTxDegrees = null;
    private Double lostTagFallbackTxDegrees = null;
    private boolean usingFallbackTx = false;
    private boolean abortRequiresReturnHome = false;
    private State abortFromState = State.IDLE;

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
                handleSpinup(dt);
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
        telemetry.addData("Aim Lost Frames", "%d/%d", lostTagFrameCount, config.turretTagLossAbortFrames);
        telemetry.addData("Aim Centered Frames", "%d/%d", centeredFrameCount, config.turretCenteredFramesRequired);
        telemetry.addData("Aim Fallback", usingFallbackTx);
        telemetry.addData("Aim Last tx", lastSeenTxDegrees == null ? "n/a" : String.format("%.2f", lastSeenTxDegrees));
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
        lostTagFrameCount = 0;
        centeredFrameCount = 0;
        lastSeenTxDegrees = null;
        lostTagFallbackTxDegrees = null;
        usingFallbackTx = false;
        abortRequiresReturnHome = false;
        abortFromState = State.IDLE;
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
        lastSeenTxDegrees = targetTxDegrees;
        lostTagFrameCount = 0;
        centeredFrameCount = 0;
        lostTagFallbackTxDegrees = null;
        usingFallbackTx = false;

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
        Double aimTxDegrees;
        if (targeting.isTargetAvailable()) {
            targetDistanceInches = targeting.getDistanceInches();
            targetTxDegrees = targeting.getTagTxDegrees();
            targetVelocity = Math.max(0.0, targeting.getTargetVelocity());
            aimTxDegrees = targetTxDegrees;
            lastSeenTxDegrees = targetTxDegrees;
            lostTagFrameCount = 0;
            lostTagFallbackTxDegrees = null;
            usingFallbackTx = false;
        } else {
            lostTagFrameCount++;
            usingFallbackTx = true;
            if (lostTagFallbackTxDegrees == null) {
                lostTagFallbackTxDegrees = lastSeenTxDegrees != null ? lastSeenTxDegrees : 0.0;
            }
            lostTagFallbackTxDegrees = functions.clamp(
                    lostTagFallbackTxDegrees,
                    -Math.abs(config.turretLostTagFallbackTxMaxDegrees),
                    Math.abs(config.turretLostTagFallbackTxMaxDegrees)
            );
            aimTxDegrees = lostTagFallbackTxDegrees;
            lostTagFallbackTxDegrees = lostTagFallbackTxDegrees * config.turretLostTagFallbackTxDecayPerFrame;

            if (lostTagFrameCount >= config.turretTagLossAbortFrames) {
                abortSequence(String.format("Lost goal tag > %d frames while aiming", config.turretTagLossAbortFrames));
                return;
            }
        }

        if (aimTxDegrees == null) {
            abortSequence("Tag tx unavailable");
            return;
        }

        boolean enforceAimTimeout = !usingFallbackTx;
        TurretCrServoController.AimResult aimResult = turret.aimToTx(aimTxDegrees, dtSeconds, config, enforceAimTimeout);
        if (aimResult == TurretCrServoController.AimResult.CENTERED) {
            centeredFrameCount++;
            if (centeredFrameCount >= config.turretCenteredFramesRequired) {
                enterState(State.SPINUP);
                status = "Turret centered stable";
            } else {
                status = String.format("Centered verify %d/%d", centeredFrameCount, config.turretCenteredFramesRequired);
            }
        } else if (aimResult == TurretCrServoController.AimResult.TIMEOUT) {
            abortSequence("Turret aim timeout");
        } else if (aimResult == TurretCrServoController.AimResult.UNAVAILABLE) {
            abortSequence("Turret unavailable");
        } else {
            double centeredHoldRange = config.turretAimDeadbandDegrees * 1.5;
            if (Math.abs(aimTxDegrees) > centeredHoldRange) {
                centeredFrameCount = 0;
            } else if (centeredFrameCount > 0) {
                centeredFrameCount--;
            }
            if (usingFallbackTx) {
                status = String.format("Aiming fallback (%d/%d lost frames)", lostTagFrameCount, config.turretTagLossAbortFrames);
            } else {
                status = "Aiming turret";
            }
        }
    }

    private void handleSpinup(double dtSeconds) {
        stopIntakes();

        boolean spinupVisionLost = false;
        if (config.turretEnableSpinupCorrections && turret != null && turret.isAvailable()) {
            targeting.update();
            Double spinupTxDegrees;
            if (targeting.isTargetAvailable()) {
                targetDistanceInches = targeting.getDistanceInches();
                targetTxDegrees = targeting.getTagTxDegrees();
                targetVelocity = Math.max(0.0, targeting.getTargetVelocity());
                spinupTxDegrees = targetTxDegrees;
                lastSeenTxDegrees = targetTxDegrees;
                lostTagFrameCount = 0;
                lostTagFallbackTxDegrees = null;
                usingFallbackTx = false;
            } else {
                spinupVisionLost = true;
                lostTagFrameCount++;
                usingFallbackTx = true;
                if (lostTagFallbackTxDegrees == null) {
                    lostTagFallbackTxDegrees = lastSeenTxDegrees != null ? lastSeenTxDegrees : 0.0;
                }
                lostTagFallbackTxDegrees = functions.clamp(
                        lostTagFallbackTxDegrees,
                        -Math.abs(config.turretLostTagFallbackTxMaxDegrees),
                        Math.abs(config.turretLostTagFallbackTxMaxDegrees)
                );
                spinupTxDegrees = lostTagFallbackTxDegrees;
                lostTagFallbackTxDegrees = lostTagFallbackTxDegrees * config.turretLostTagFallbackTxDecayPerFrame;
            }

            boolean correctionAllowed = !spinupVisionLost || lostTagFrameCount < config.turretTagLossAbortFrames;
            if (spinupTxDegrees != null && correctionAllowed) {
                TurretCrServoController.AimResult spinupAim = turret.aimToTx(spinupTxDegrees, dtSeconds, config, false);
                if (spinupAim == TurretCrServoController.AimResult.UNAVAILABLE) {
                    abortSequence("Turret unavailable during spinup");
                    return;
                }
            } else {
                turret.stop();
            }
        }

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
            if (!timeoutMet && requiresPreFeedRealign()) {
                centeredFrameCount = 0;
                if (turret != null && turret.isAvailable()) turret.beginAimAttempt();
                enterState(State.AIM);
                status = "Re-aim before feed";
                return;
            }
            enterState(activeSequence == SequenceType.THREE_BALL ? State.FEED_THREE_BACKOFF : State.FEED_TWO);
            status = settleMet ? "Spinup ready" : "Spinup timeout fallback";
        } else {
            if (spinupVisionLost) {
                status = String.format("Spinning up (vision lost %d/%d)", lostTagFrameCount, config.turretTagLossAbortFrames);
            } else {
                status = "Spinning up";
            }
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
            status = abortReason.isEmpty() ? "Returning turret home" : "Returning home after abort";
        }
    }

    private void handleAbort() {
        stopIntakes();
        setFlywheelVelocity(config.flywheelIdleVelocity);
        if (abortRequiresReturnHome && turret != null && turret.isAvailable()) {
            enterState(State.RETURN_HOME);
            return;
        }

        if (turret != null) turret.resetHomeModel();
        enterState(State.COMPLETE);
        status = "Abort complete";
    }

    private void finishSequence() {
        stopIntakes();
        setFlywheelVelocity(config.flywheelIdleVelocity);
        activeSequence = SequenceType.NONE;
        state = State.IDLE;
        status = abortReason.isEmpty() ? "Idle" : "Idle after abort";
    }

    private void abortSequence(String reason) {
        abortFromState = state;
        abortRequiresReturnHome = shouldReturnHomeAfterAbort(abortFromState);
        abortReason = String.format("%s (from %s)", reason, abortFromState);
        status = reason;
        enterState(State.ABORT);
    }

    private void enterState(State newState) {
        state = newState;
        stateTimer.reset();
        if (newState == State.AIM) {
            centeredFrameCount = 0;
        }
        if (newState == State.SPINUP) {
            readyWindowActive = false;
        }
        if (newState == State.RETURN_HOME && turret != null && turret.isAvailable()) {
            turret.clampEstimatedTurnOffset(config.turretReturnMaxEstimatedOffset);
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

    private boolean requiresPreFeedRealign() {
        if (usingFallbackTx) return true;
        Double tx = targetTxDegrees != null ? targetTxDegrees : lastSeenTxDegrees;
        if (tx == null) return true;
        return Math.abs(tx) > config.preFeedMaxTxDegrees;
    }

    private boolean shouldReturnHomeAfterAbort(State fromState) {
        return fromState == State.FEED_TWO
                || fromState == State.FEED_THREE_BACKOFF
                || fromState == State.FEED_THREE_FORWARD;
    }
}
