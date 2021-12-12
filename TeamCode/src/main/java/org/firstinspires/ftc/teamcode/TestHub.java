package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;

@TeleOp(name = "TestHub", group = "Test")
public class TestHub extends LinearOpMode {

    // An array of motors so we can test each motor controller port
    DcMotorEx[] motors = new DcMotorEx[4];

    // An array of servos so we can test each servo controller port
    Servo[] servos = new Servo[6];

    // The gyro built in to the expansion hub
    BNO055IMU imu;

    // A Rev 2m Distance Sensor
    DistanceSensor distSensor;

    // A potentiometer to provide analog voltage input
    AnalogInput analogInput;

    // A touch sensor to provide digital input
    DigitalChannel digitalChannel;

    // Button Toggle objects that we hook up to gamepad1.dpad_up and gamepad1.dpad_down,
    // as well as gamepad1.a and gamepad1.y
    ButtonToggle toggleDPadUp = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.dpad_up;
        }
    };

    ButtonToggle toggleDPadDown = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.dpad_down;
        }
    };

    ButtonToggle toggleA = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };

    ButtonToggle toggleY = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.y;
        }
    };


    public void runOpMode(){

        motors[0] = hardwareMap.get(DcMotorEx.class, "m0");
        motors[1] = hardwareMap.get(DcMotorEx.class, "m1");
        motors[2] = hardwareMap.get(DcMotorEx.class, "m2");
        motors[3] = hardwareMap.get(DcMotorEx.class, "m3");

        for (int i=0; i<4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        servos[0] = hardwareMap.get(Servo.class, "s0");
        servos[1] = hardwareMap.get(Servo.class, "s1");
        servos[2] = hardwareMap.get(Servo.class, "s2");
        servos[3] = hardwareMap.get(Servo.class, "s3");
        servos[4] = hardwareMap.get(Servo.class, "s4");
        servos[5] = hardwareMap.get(Servo.class, "s5");

        for (int i=0; i<6; i++){
            servos[i].setPosition(0);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BN055Cali.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        distSensor = hardwareMap.get(DistanceSensor.class, "distance");
        analogInput = hardwareMap.get(AnalogInput.class, "analog");
        digitalChannel = hardwareMap.get(DigitalChannel.class, "digital");

        int motorIndex = 0;
        int servoIndex = 0;

        waitForStart();

        while (opModeIsActive()){
            if (toggleDPadUp.update()) motorIndex = (motorIndex + 1) % 4;
            if (toggleDPadDown.update()) servoIndex = (servoIndex + 1) % 6;

            motors[motorIndex].setPower(-gamepad1.left_stick_y);

            double servoPos = servos[servoIndex].getPosition();
            if (toggleA.update()) {
                servoPos -= 0.25;
                servoPos = Range.clip(servoPos, 0, 1);
                servos[servoIndex].setPosition(servoPos);
            }

            if (toggleY.update()){
                servoPos += 0.25;
                servoPos = Range.clip(servoPos, 0, 1);
                servos[servoIndex].setPosition(servoPos);
            }

            telemetry.addData("Motor", "%d: Pwr = %.2f  Ticks = %d",
                    motorIndex, motors[motorIndex].getPower(), motors[motorIndex].getCurrentPosition());
            telemetry.addData("Servo", "%d: Position = %.2f", servoIndex,
                    servos[servoIndex].getPosition());
            telemetry.addData("Distance", distSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Voltage", analogInput.getVoltage());
            telemetry.addData("Touch", !digitalChannel.getState());

            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES);
            telemetry.addData("Angles", "Hd = %.1f  P = %.1f  R = %.1f",
                    orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle);

            telemetry.update();

        }




    }

}
