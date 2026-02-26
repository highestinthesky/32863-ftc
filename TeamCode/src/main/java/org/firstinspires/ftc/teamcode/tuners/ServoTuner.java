package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "ServoTuner", group = "Tuners")
public class ServoTuner extends OpMode {
    public static String SERVO_NAME = "tservo";
    public static boolean CONTINUOUS_SERVO = false;
    public static boolean REVERSE = false;

    public static class BoundServo {
        public final Servo servo;
        public final CRServo crServo;
        public final boolean continuous;

        BoundServo(Servo servo, CRServo crServo, boolean continuous) {
            this.servo = servo;
            this.crServo = crServo;
            this.continuous = continuous;
        }
    }

    private BoundServo boundServo;
    private String initError;

    public static BoundServo bindServo(HardwareMap hardwareMap, String servoName, boolean continuousServo) {
        if (continuousServo) {
            return new BoundServo(null, hardwareMap.get(CRServo.class, servoName), true);
        }
        return new BoundServo(hardwareMap.get(Servo.class, servoName), null, false);
    }

    public static void runMax(BoundServo boundServo, boolean reverse) {
        if (boundServo == null) return;
        if (boundServo.continuous) {
            boundServo.crServo.setPower(reverse ? -1.0 : 1.0);
        } else {
            boundServo.servo.setPosition(reverse ? 0.0 : 1.0);
        }
    }

    public static void stopServo(BoundServo boundServo) {
        if (boundServo == null) return;
        if (boundServo.continuous) {
            boundServo.crServo.setPower(0.0);
        }
    }

    @Override
    public void init() {
        try {
            boundServo = bindServo(hardwareMap, SERVO_NAME, CONTINUOUS_SERVO);
            initError = null;
        } catch (Exception e) {
            boundServo = null;
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
        printTelemetry(telemetry, SERVO_NAME, boundServo, initError, REVERSE);
    }

    @Override
    public void loop() {
        if (boundServo != null) {
            runMax(boundServo, REVERSE);
        }
        printTelemetry(telemetry, SERVO_NAME, boundServo, initError, REVERSE);
    }

    @Override
    public void stop() {
        stopServo(boundServo);
    }

    public static void printTelemetry(Telemetry telemetry, String servoName, BoundServo boundServo, String error, boolean reverse) {
        telemetry.addLine("Servo Tuner");
        telemetry.addData("Servo Name", servoName);
        telemetry.addData("Continuous", boundServo != null ? boundServo.continuous : CONTINUOUS_SERVO);
        telemetry.addData("Reverse", reverse);
        if (error != null) {
            telemetry.addData("Status", "ERROR");
            telemetry.addData("Reason", error);
        } else if (boundServo == null) {
            telemetry.addData("Status", "No servo");
        } else if (boundServo.continuous) {
            telemetry.addData("Status", "Running max power");
            telemetry.addData("Target Power", reverse ? -1.0 : 1.0);
        } else {
            telemetry.addData("Status", "Holding max position");
            telemetry.addData("Target Position", reverse ? 0.0 : 1.0);
        }
        telemetry.update();
    }
}
