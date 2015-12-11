package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.AdafruitIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class zDemoTank extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    Servo jjDumper;
    Servo redNoodle;
    Servo blueNoodle;

    AdafruitIMU boschBNO055;



    @Override
    public void init() {

        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        double leftY = -gamepad1.left_stick_y;
        double rightY = -gamepad1.right_stick_y;

        double leftSquaredVal = leftY * leftY;
        double rightSquaredVal = rightY * rightY;

        double leftPower;
        double rightPower;

        if(leftY < 0) {
            leftPower = -leftSquaredVal;
        } else {
            leftPower = leftSquaredVal;
        }

        if(rightY < 0) {
            rightPower = -rightSquaredVal;
        } else {
            rightPower = rightSquaredVal;
        }

        leftMotor.setPower(leftPower);

        rightMotor.setPower(rightPower);

    }
}
