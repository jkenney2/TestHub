package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.ButtonToggle;

public class TestHub extends LinearOpMode {

    DcMotorEx[] motors = new DcMotorEx[4];
    Servo[] servos = new Servo[6];
    BNO055IMU imu;

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

    public void runOpMode(){

        motors[0] = hardwareMap.get(DcMotorEx.class, "m0");
        motors[1] = hardwareMap.get(DcMotorEx.class, "m1");
        motors[2] = hardwareMap.get(DcMotorEx.class, "m2");
        motors[3] = hardwareMap.get(DcMotorEx.class, "m3");

        servos[0] = hardwareMap.get(Servo.class, "s0");
        servos[1] = hardwareMap.get(Servo.class, "s1");
        servos[2] = hardwareMap.get(Servo.class, "s2");
        servos[3] = hardwareMap.get(Servo.class, "s3");
        servos[4] = hardwareMap.get(Servo.class, "s4");
        servos[5] = hardwareMap.get(Servo.class, "s5");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BN055Cali.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        int motorIndex = 0;
        int servoIndex = 0;

        for (int i=0; i<6; i++){
            servos[i].setPosition(0);
        }

        while (opModeIsActive()){
            if (toggleDPadUp.update()) motorIndex = (motorIndex + 1) % 4;
            if (toggleDPadDown.update()) servoIndex = (servoIndex + 1) % 6;

            motors[motorIndex].setPower(-gamepad1.left_stick_y);

            double servoPos = servos[servoIndex].getPosition();
            if (gamepad1.a) servoPos -= 0.002;
            else if (gamepad1.x) servoPos += 0.002;
            servoPos = Range.clip(servoPos, 0, 1);
            servos[servoIndex].setPosition(servoPos);



        }




    }

}
