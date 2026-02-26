package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FaultTolerantMecanumDrive {
    private final DcMotorEx leftFrontDrive;
    private final DcMotorEx rightFrontDrive;
    private final DcMotorEx leftBackDrive;
    private final DcMotorEx rightBackDrive;
    private final Telemetry telemetry;

    private double prevMax = 0.275;
    private String faultEventMsg = "";
    private int faultEventLoopsLeft = 0;
    private boolean stopping = false;

    private static final double CMD_MIN = 0.35;
    private static final double VEL_ABS_MIN = 120.0;
    private static final double VEL_REL_RATIO = 0.20;
    private static final int FAIL_LOOPS = 8;
    private static final int RECOVER_LOOPS = 15;

    private static class MotorHealth {
        int badLoops = 0;
        int goodLoops = 0;
        boolean failed = false;
    }

    public static final class MotorSafety {
        private MotorSafety() {}

        public static void setPower(DcMotor motor, double power) {
            if (motor == null) return;
            try {
                motor.setPower(power);
            } catch (Exception ignored) {
            }
        }

        public static void setVelocity(DcMotorEx motor, double velocity) {
            if (motor == null) return;
            try {
                motor.setVelocity(velocity);
            } catch (Exception ignored) {
                try {
                    motor.setPower(0);
                } catch (Exception ignored2) {
                }
            }
        }

        public static void setZeroPowerBehavior(DcMotor motor, DcMotor.ZeroPowerBehavior behavior) {
            if (motor == null) return;
            try {
                motor.setZeroPowerBehavior(behavior);
            } catch (Exception ignored) {
            }
        }
    }

    private final MotorHealth lfHealth = new MotorHealth();
    private final MotorHealth rfHealth = new MotorHealth();
    private final MotorHealth lbHealth = new MotorHealth();
    private final MotorHealth rbHealth = new MotorHealth();

    public FaultTolerantMecanumDrive(
            DcMotorEx leftFrontDrive,
            DcMotorEx rightFrontDrive,
            DcMotorEx leftBackDrive,
            DcMotorEx rightBackDrive,
            Telemetry telemetry) {
        this.leftFrontDrive = leftFrontDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightBackDrive = rightBackDrive;
        this.telemetry = telemetry;
    }

    public void resetForStart() {
        stopping = false;
        prevMax = 0.275;
        faultEventMsg = "";
        faultEventLoopsLeft = 0;
    }

    public void stopSafely() {
        stopping = true;
        MotorSafety.setPower(leftFrontDrive, 0);
        MotorSafety.setPower(rightFrontDrive, 0);
        MotorSafety.setPower(leftBackDrive, 0);
        MotorSafety.setPower(rightBackDrive, 0);

        MotorSafety.setZeroPowerBehavior(leftFrontDrive, DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSafety.setZeroPowerBehavior(rightFrontDrive, DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSafety.setZeroPowerBehavior(leftBackDrive, DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSafety.setZeroPowerBehavior(rightBackDrive, DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void updateHealth(MotorHealth h, DcMotorEx m, double cmd, double vel, double maxVel) {
        if (stopping) return;
        if (Math.abs(cmd) < CMD_MIN || maxVel < VEL_ABS_MIN) {
            h.badLoops = 0;
            h.goodLoops = 0;
            return;
        }

        double threshold = Math.max(VEL_ABS_MIN, maxVel * VEL_REL_RATIO);
        boolean moving = vel >= threshold;

        if (!moving) {
            h.badLoops++;
            h.goodLoops = 0;
            if (h.badLoops >= FAIL_LOOPS) {
                if (!h.failed) MotorSafety.setZeroPowerBehavior(m, DcMotor.ZeroPowerBehavior.FLOAT);
                h.failed = true;
            }
        } else {
            h.goodLoops++;
            h.badLoops = 0;
            if (h.failed && h.goodLoops >= RECOVER_LOOPS) {
                h.failed = false;
                MotorSafety.setZeroPowerBehavior(m, DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    public void move(double drive, double strafe, double turn) {
        if (stopping) return;

        boolean prevLfFailed = lfHealth.failed;
        boolean prevRfFailed = rfHealth.failed;
        boolean prevLbFailed = lbHealth.failed;
        boolean prevRbFailed = rbHealth.failed;

        double vLF = Math.abs(leftFrontDrive.getVelocity());
        double vRF = Math.abs(rightFrontDrive.getVelocity());
        double vLB = Math.abs(leftBackDrive.getVelocity());
        double vRB = Math.abs(rightBackDrive.getVelocity());
        double maxV = Math.max(Math.max(vLF, vRF), Math.max(vLB, vRB));

        double lf = drive - strafe - turn;
        double rf = drive + strafe + turn;
        double lb = drive + strafe - turn;
        double rb = drive - strafe + turn;

        double detMax = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (detMax > 1.0) {
            lf /= detMax;
            rf /= detMax;
            lb /= detMax;
            rb /= detMax;
        }

        updateHealth(lfHealth, leftFrontDrive, lf, vLF, maxV);
        updateHealth(rfHealth, rightFrontDrive, rf, vRF, maxV);
        updateHealth(lbHealth, leftBackDrive, lb, vLB, maxV);
        updateHealth(rbHealth, rightBackDrive, rb, vRB, maxV);

        boolean lfOk = !lfHealth.failed;
        boolean rfOk = !rfHealth.failed;
        boolean lbOk = !lbHealth.failed;
        boolean rbOk = !rbHealth.failed;

        int failedCount = (lfOk ? 0 : 1) + (rfOk ? 0 : 1) + (lbOk ? 0 : 1) + (rbOk ? 0 : 1);

        if (!prevLfFailed && lfHealth.failed) {
            faultEventMsg = "Front Left motor is currently offline and the system has adjusted.";
            faultEventLoopsLeft = 30;
        } else if (!prevRfFailed && rfHealth.failed) {
            faultEventMsg = "Front Right motor is currently offline and the system has adjusted.";
            faultEventLoopsLeft = 30;
        } else if (!prevLbFailed && lbHealth.failed) {
            faultEventMsg = "Back Left motor is currently offline and the system has adjusted.";
            faultEventLoopsLeft = 30;
        } else if (!prevRbFailed && rbHealth.failed) {
            faultEventMsg = "Back Right motor is currently offline and the system has adjusted.";
            faultEventLoopsLeft = 30;
        }

        if (failedCount == 1) {
            double d = drive;
            double s = strafe;
            double t = turn;

            if (!lfOk) {
                lf = 0;
                rf = 2 * (s + t);
                lb = 2 * (d - t);
                rb = 2 * (d - s);
            } else if (!rfOk) {
                rf = 0;
                lf = -2 * (s + t);
                lb = 2 * (d + s);
                rb = 2 * (d + t);
            } else if (!lbOk) {
                lb = 0;
                lf = 2 * (d - t);
                rf = 2 * (d + s);
                rb = 2 * (t - s);
            } else {
                rb = 0;
                lf = 2 * (d - s);
                rf = 2 * (d + t);
                lb = 2 * (s - t);
            }
        }

        double maxAbs = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (maxAbs > 1.0) {
            lf /= maxAbs;
            rf /= maxAbs;
            lb /= maxAbs;
            rb /= maxAbs;
            maxAbs = 1.0;
        }

        double desired = maxAbs;
        double allowed = desired;

        if (allowed > prevMax + 0.075) allowed = prevMax + 0.075;
        if (allowed < 0) allowed = 0;
        if (allowed > 1) allowed = 1;

        double scale = (desired > 1e-6) ? (allowed / desired) : 0.0;
        prevMax = allowed;

        lf *= scale;
        rf *= scale;
        lb *= scale;
        rb *= scale;

        MotorSafety.setPower(leftFrontDrive, lf);
        MotorSafety.setPower(rightFrontDrive, rf);
        MotorSafety.setPower(leftBackDrive, lb);
        MotorSafety.setPower(rightBackDrive, rb);

        if (faultEventLoopsLeft > 0) {
            telemetry.addLine(faultEventMsg);
            faultEventLoopsLeft--;
        }

        if (failedCount > 0) {
            String offline =
                    (!lfOk ? "Front Left" :
                            !rfOk ? "Front Right" :
                                    !lbOk ? "Back Left" :
                                            "Back Right");
            telemetry.addLine(offline + " motor is currently offline (fault mode active).");
        }

        telemetry.addData("DriveVel", "LF %.0f RF %.0f LB %.0f RB %.0f", vLF, vRF, vLB, vRB);
    }
}
