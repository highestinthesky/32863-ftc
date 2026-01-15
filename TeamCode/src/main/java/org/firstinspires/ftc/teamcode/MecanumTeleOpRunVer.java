package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="MecanumTeleOpRunVer")
public class MecanumTeleOpRunVer extends OpMode {

    // Drive motors
    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;

    // Other motors/servos
    private DcMotor intake;
    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;
    private DcMotorEx rampwheel;

    private Servo lspindexerup;
    private Servo rspindexerup;
    private double tservoPosition = 0.0;

    // Optional ramp limiter state
    private static double prevMax = 0.275;
    public double highVelocity = 6000;
    public double lowVelocity = 1500;
    double curTargetVelocity = highVelocity;
    public double basespindexerup = 180;
    public double basespindexerdown = 70;
    double curTargetspindexer = basespindexerdown;
    double rampwheelOn = 1150;
    double rampwheelOff = 0;
    double rampwheelbackwards = -300;

    // Telemetry notice when a motor *first* fails
    private String faultEventMsg = "";
    private int faultEventLoopsLeft = 0;
    private boolean stopping = false;



//    double F = 0;
//    double P = 0;
//    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.001};
//    int stepIndex = 1;

    // Timer for temporary indexer movement
    private ElapsedTime indexerAndRampWheelTimer = new ElapsedTime();
    private boolean indexerAndRampWheelTimerActive = false;
    private boolean overrideMode = false;

    // Pinpoint placeholder
    private PinpointIO pinpoint;

    @Override
    public void init() {
        // chassis motors setup
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        // chassis motors direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // chassis brake modes
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // chassis encoder setup
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // intake setup call
        intake = hardwareMap.get(DcMotor.class, "intake");
        // intake directions setup
        intake.setDirection(DcMotor.Direction.FORWARD);
        // intake encoder setup
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // flywheel setup call
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");
        // flywheel directions
        rightFlyWheel.setDirection(DcMotorEx.Direction.FORWARD);
        leftFlyWheel.setDirection(DcMotorEx.Direction.REVERSE);
        // flywheel encoder setup
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // flywheel PIDF setup
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(12,0,0,2);
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // rampwheel setup call
        rampwheel = hardwareMap.get(DcMotorEx.class, "turretturn");
        // rampwheel directions
        rampwheel.setDirection(DcMotorEx.Direction.REVERSE);
        // rampwheel encoder setup
        rampwheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // rampwheel PIDF coefficients setup
        PIDFCoefficients rampwheelcoefficients = new PIDFCoefficients(4,0,0,20);
        rampwheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, rampwheelcoefficients);

        // tservo setup call
        Servo tservo = hardwareMap.get(Servo.class, "tservo");
        // tservo directions
        tservo.setDirection(Servo.Direction.REVERSE);

        //spindexerup setup calls
        lspindexerup = hardwareMap.get(Servo.class, "lspindexerup");
        rspindexerup = hardwareMap.get(Servo.class, "rspindexerup");
        // spindexerup directions
        lspindexerup.setDirection(Servo.Direction.FORWARD);
        rspindexerup.setDirection(Servo.Direction.FORWARD);

        // ---- Pinpoint (placeholder) ----
        pinpoint = new PinpointIO(hardwareMap, telemetry, "odo");
        pinpoint.initializeAndConfigure();

        telemetry.addLine("Initialized. Press PLAY.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Set initial servo positions to 70 degrees
        setServoAngle(lspindexerup, 70);
        setServoAngle(rspindexerup, 70);
        prevMax = 0.275; // reset ramp at start too
    }

    @Override
    public void loop() {
        if (stopping) return;
        pinpoint.update();
//        int action = 0;
//        if (gamepad1.bWasPressed()) action = 2;
//        else if (gamepad1.dpadLeftWasPressed()) action = 3;
//        else if (gamepad1.dpadRightWasPressed()) action = 4;
//        else if (gamepad1.dpadUpWasPressed()) action = 5;
//        else if (gamepad1.dpadDownWasPressed()) action = 6;
//
//        switch (action) {
//            case 2:
//                stepIndex = (stepIndex + 1) % stepSizes.length;
//                break;
//            case 3:
//                F -= stepSizes[stepIndex];
//                break;
//            case 4:
//                F += stepSizes[stepIndex];
//                break;
//            case 5:
//                P += stepSizes[stepIndex];
//                break;
//            case 6:
//                P -= stepSizes[stepIndex];
//                break;
//        }
        //Flywheel controlling velocity
        if (gamepad1.yWasPressed()) curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;

        // activating overrideMode
        if (gamepad2.aWasPressed() && !gamepad2.aWasReleased()) {overrideMode = true;}
        else {overrideMode = false;}

        // code to change rampwheel to go backwards
        if (overrideMode && gamepad2.right_trigger > 0.1) {
            rampwheel.setVelocity(rampwheelbackwards);
        } else if (overrideMode){
            rampwheel.setVelocity(rampwheelOff);
        }

        // --- Indexer control + rampWheel control (gamepad2 B) ---
        if (gamepad2.bWasPressed()) {
            PIDFCoefficients rampWheelCoefficients = new PIDFCoefficients(4,0,0,20);
            rampwheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, rampWheelCoefficients);
            rampwheel.setVelocity(rampwheelOn);
            curTargetspindexer = basespindexerup;
            indexerAndRampWheelTimer.reset();
            indexerAndRampWheelTimerActive = true;
        }
        if (indexerAndRampWheelTimerActive && indexerAndRampWheelTimer.seconds() >= 0.3 && indexerAndRampWheelTimer.seconds() <= 0.6) {
            curTargetspindexer = basespindexerdown;
        }
        if (indexerAndRampWheelTimerActive && indexerAndRampWheelTimer.seconds() >= 0.8) {
            rampwheel.setVelocity(rampwheelOff);
            indexerAndRampWheelTimerActive = false;
        }
        setServoAngle(lspindexerup, curTargetspindexer);
        setServoAngle(rspindexerup, curTargetspindexer);


//         --- Turret rotation (gamepad2 bumpers) ---
//        int turretState = 0;
//        if (gamepad2.right_bumper) turretState = 1;
//        else if (gamepad2.left_bumper) turretState = 2;
//
//        switch (turretState) {
//            case 1:
//                turretRotation.setPower(1);
//                break;
//            case 2:
//                turretRotation.setPower(-1);
//                break;
//            default:
//                turretRotation.setPower(0);
//                break;
//        }

        // --- T-Servo incremental control (gamepad2 X/Y) ---
//        if (gamepad2.x) {
//            tservoPosition += 0.1;
//        } else if (gamepad2.y) {
//            tservoPosition -= 0.1;
//        }

//        if (tservoPosition > 1.0) tservoPosition = 1.0;
//        else if (tservoPosition < 0.0) tservoPosition = 0.0;
//
//        tservo.setPosition(tservoPosition);

        // --- Intake (gamepad2 left trigger) ---
        if (gamepad2.left_trigger > 0.1) {
            if (overrideMode) {
                intake.setPower(-1);
            } else {
                intake.setPower(1);
            }
        } else {
            intake.setPower(0);
        }
        
        if (gamepad2.right_bumper) {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(12,0,0,4);
            leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            leftFlyWheel.setVelocity(curTargetVelocity);
            rightFlyWheel.setVelocity(curTargetVelocity);
        } else{
            leftFlyWheel.setVelocity(0);
            rightFlyWheel.setVelocity(0);
        }

        // --- Drive control ---
        double drive  = -gamepad1.left_stick_y;        
        double strafe = -gamepad1.left_stick_x;        
        double turn   = -gamepad1.right_stick_x;

        moveRobotFaultTolerant(drive, strafe, turn);
        

        telemetry.addLine("--------------------------------------");
        telemetry.addData("T-Servo Position", "%.3f", tservoPosition);
        telemetry.addData("Override Mode", overrideMode);
        telemetry.addData("Odo X (in)", "%.1f", pinpoint.getXInches());
        telemetry.addData("Odo Y (in)", "%.1f", pinpoint.getYInches());
        telemetry.addData("Odo H (deg)", "%.1f", Math.toDegrees(pinpoint.getHeadingRadians()));
        telemetry.update();
    }

    @Override
    public void stop() {
        stopping = true;

        // Stop velocity-controlled motors first
        safeSetVelocity(leftFlyWheel, 0);
        safeSetVelocity(rightFlyWheel, 0);
        safeSetVelocity(rampwheel, 0);

        // Stop everything else
        safeSetPower(leftFrontDrive, 0);
        safeSetPower(rightFrontDrive, 0);
        safeSetPower(leftBackDrive, 0);
        safeSetPower(rightBackDrive, 0);
        safeSetPower(intake, 0);

        // In case fault mode set one motor to FLOAT, restore BRAKE
        safeSetZeroPowerBehavior(leftFrontDrive, DcMotor.ZeroPowerBehavior.BRAKE);
        safeSetZeroPowerBehavior(rightFrontDrive, DcMotor.ZeroPowerBehavior.BRAKE);
        safeSetZeroPowerBehavior(leftBackDrive, DcMotor.ZeroPowerBehavior.BRAKE);
        safeSetZeroPowerBehavior(rightBackDrive, DcMotor.ZeroPowerBehavior.BRAKE);

    }
    private void safeSetPower(DcMotor m, double p) {
        if (m == null) return;
        try { m.setPower(p); } catch (Exception ignored) {}
    }

    private void safeSetVelocity(DcMotorEx m, double v) {
        if (m == null) return;
        try { m.setVelocity(v); }
        catch (Exception ignored) {
            try { m.setPower(0); } catch (Exception ignored2) {}
        }
    }

    private void safeSetZeroPowerBehavior(DcMotor m, DcMotor.ZeroPowerBehavior b) {
        if (m == null) return;
        try { m.setZeroPowerBehavior(b); } catch (Exception ignored) {}
    }
    /**
     * Sets a servo position based on degrees (0-300 for "full range" servos).
     */
    public void setServoAngle(Servo servo, double angle) {
        servo.setPosition(angle / 300.0);
    }

//    public void moveRobot(double drive, double strafe, double turn) {
//        double leftFrontPower  = drive - strafe - turn;
//        double rightFrontPower = drive + strafe + turn;
//        double leftBackPower   = drive + strafe - turn;
//        double rightBackPower  = drive - strafe + turn;
//
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > prevMax + 0.075) {
//            max = prevMax + 0.075;
//            prevMax = max;
//
//            leftFrontPower  *= max;
//            rightFrontPower *= max;
//            leftBackPower   *= max;
//            rightBackPower  *= max;
//        } else if (max > 1.0) {
//            leftFrontPower  /= max;
//            rightFrontPower /= max;
//            leftBackPower   /= max;
//            rightBackPower  /= max;
//        } else {
//            prevMax = max;
//        }
//
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//    }

    private static final double CMD_MIN = 0.35;
    private static final double VEL_ABS_MIN = 120.0;  // ticks/sec
    private static final double VEL_REL_RATIO = 0.20;
    private static final int FAIL_LOOPS = 8;
    private static final int RECOVER_LOOPS = 15;

    private static class MotorHealth {
        int badLoops = 0;
        int goodLoops = 0;
        boolean failed = false;
    }

    private final MotorHealth lfHealth = new MotorHealth();
    private final MotorHealth rfHealth = new MotorHealth();
    private final MotorHealth lbHealth = new MotorHealth();
    private final MotorHealth rbHealth = new MotorHealth();

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
                if (!h.failed) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                h.failed = true;
            }
        } else {
            h.goodLoops++;
            h.badLoops = 0;
            if (h.failed && h.goodLoops >= RECOVER_LOOPS) {
                h.failed = false;
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    public void moveRobotFaultTolerant(double drive, double strafe, double turn) {
        // Remember previous failure states (so we can detect "just failed")
        boolean prevLfFailed = lfHealth.failed;
        boolean prevRfFailed = rfHealth.failed;
        boolean prevLbFailed = lbHealth.failed;
        boolean prevRbFailed = rbHealth.failed;

        // --- Measure current velocities ---
        double vLF = Math.abs(leftFrontDrive.getVelocity());
        double vRF = Math.abs(rightFrontDrive.getVelocity());
        double vLB = Math.abs(leftBackDrive.getVelocity());
        double vRB = Math.abs(rightBackDrive.getVelocity());
        double maxV = Math.max(Math.max(vLF, vRF), Math.max(vLB, vRB));

        // --- Normal 4-wheel powers (your exact mapping) ---
        double lf = drive - strafe - turn;
        double rf = drive + strafe + turn;
        double lb = drive + strafe - turn;
        double rb = drive - strafe + turn;

        // Normalize these for detection consistency
        double detMax = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (detMax > 1.0) {
            lf /= detMax; rf /= detMax; lb /= detMax; rb /= detMax;
        }

        // --- Update health flags ---
        updateHealth(lfHealth, leftFrontDrive,  lf, vLF, maxV);
        updateHealth(rfHealth, rightFrontDrive, rf, vRF, maxV);
        updateHealth(lbHealth, leftBackDrive,   lb, vLB, maxV);
        updateHealth(rbHealth, rightBackDrive,  rb, vRB, maxV);

        boolean lfOk = !lfHealth.failed;
        boolean rfOk = !rfHealth.failed;
        boolean lbOk = !lbHealth.failed;
        boolean rbOk = !rbHealth.failed;

        int failedCount = (lfOk ? 0 : 1) + (rfOk ? 0 : 1) + (lbOk ? 0 : 1) + (rbOk ? 0 : 1);

        // --- If a motor JUST failed this loop, create the requested telemetry message ---
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

        // --- If exactly ONE motor failed, remap to 3-wheel solution ---
        if (failedCount == 1) {
            double d = drive, s = strafe, t = turn;

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
            } else { // !rbOk
                rb = 0;
                lf = 2 * (d - s);
                rf = 2 * (d + t);
                lb = 2 * (s - t);
            }
        }

        // --- Normalize final outputs ---
        double maxAbs = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (maxAbs > 1.0) {
            lf /= maxAbs; rf /= maxAbs; lb /= maxAbs; rb /= maxAbs;
            maxAbs = 1.0;
        }

        // --- Keep your ramp limiter behavior, but apply it correctly ---
        double desired = maxAbs; // 0..1
        double allowed = desired;

        if (allowed > prevMax + 0.075) allowed = prevMax + 0.075;
        if (allowed < 0) allowed = 0;
        if (allowed > 1) allowed = 1;

        double scale = (desired > 1e-6) ? (allowed / desired) : 0.0;
        prevMax = allowed;

        lf *= scale; rf *= scale; lb *= scale; rb *= scale;

        // --- Apply powers ---
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);

        // --- Telemetry output ---
        if (faultEventLoopsLeft > 0) {
            telemetry.addLine(faultEventMsg);
            faultEventLoopsLeft--;
        }

        // Persistent status while any motor is failed
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
    private static class PinpointIO {

        private final GoBildaPinpointDriver odo;
        private final Telemetry telemetry;

        PinpointIO(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
            this.telemetry = telemetry;
            this.odo = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
        }

        void initializeAndConfigure() {
            // MUST be done at init/start while robot is still
            odo.resetPosAndIMU();
            telemetry.addLine("Pinpoint initialized");
            telemetry.update();
        }

        void update() {
            odo.update();
        }
        double getXInches() {
            return odo.getPosition().getX(DistanceUnit.INCH);
        }
        double getYInches() {
            return odo.getPosition().getY(DistanceUnit.INCH);
        }
        double getHeadingRadians() {
            return odo.getPosition().getHeading(AngleUnit.RADIANS);
        }
    }
}
