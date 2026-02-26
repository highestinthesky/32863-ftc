package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TunerCall", group = "Tuners")
public class TunerCall extends OpMode {
    public enum TunerMode {
        DC_MOTOR,
        SERVO
    }

    public static TunerMode MODE = TunerMode.DC_MOTOR;
    public static String DEVICE_NAME = "frontLeftMotor";
    public static boolean REVERSE = false;

    // Set this true when MODE == SERVO and the device is a CRServo.
    public static boolean SERVO_IS_CONTINUOUS = false;

    private DcMotor boundMotor;
    private ServoTuner.BoundServo boundServo;
    private String initError;

    @Override
    public void init() {
        boundMotor = null;
        boundServo = null;
        initError = null;

        try {
            switch (MODE) {
                case DC_MOTOR:
                    boundMotor = DcMotorTuner.bindMotor(hardwareMap, DEVICE_NAME);
                    DcMotorTuner.configureMotor(boundMotor, false);
                    break;
                case SERVO:
                    boundServo = ServoTuner.bindServo(hardwareMap, DEVICE_NAME, SERVO_IS_CONTINUOUS);
                    break;
            }
        } catch (Exception e) {
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }

        updateTunerTelemetry();
    }

    @Override
    public void loop() {
        if (initError == null) {
            switch (MODE) {
                case DC_MOTOR:
                    DcMotorTuner.runMax(boundMotor, REVERSE);
                    break;
                case SERVO:
                    ServoTuner.runMax(boundServo, REVERSE);
                    break;
            }
        }

        updateTunerTelemetry();
    }

    @Override
    public void stop() {
        DcMotorTuner.stopMotor(boundMotor);
        ServoTuner.stopServo(boundServo);
    }

    private void updateTunerTelemetry() {
        telemetry.addLine("TunerCall");
        telemetry.addData("Mode", MODE);
        telemetry.addData("Device Name", DEVICE_NAME);
        telemetry.addData("Reverse", REVERSE);
        if (MODE == TunerMode.SERVO) telemetry.addData("CR Servo", SERVO_IS_CONTINUOUS);

        if (initError != null) {
            telemetry.addData("Status", "ERROR");
            telemetry.addData("Reason", initError);
            telemetry.update();
            return;
        }

        switch (MODE) {
            case DC_MOTOR:
                telemetry.addData("Action", "Calling DcMotorTuner.runMax()");
                telemetry.addData("Power", REVERSE ? -1.0 : 1.0);
                if (boundMotor != null) telemetry.addData("Encoder", boundMotor.getCurrentPosition());
                break;
            case SERVO:
                telemetry.addData("Action", "Calling ServoTuner.runMax()");
                if (boundServo != null && boundServo.continuous) {
                    telemetry.addData("Power", REVERSE ? -1.0 : 1.0);
                } else {
                    telemetry.addData("Position", REVERSE ? 0.0 : 1.0);
                }
                break;
        }
        telemetry.update();
    }
}
