package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class zDemoArcade extends OpMode {

    final double UP_POSITION = 0.0;
    final double DOWN_POSITION = 1.0;

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init() {

        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        double xValue = -gamepad1.left_stick_x;
        double yValue = -gamepad1.left_stick_y;

        double xSquaredVal = xValue * xValue;
        double ySquaredVal = yValue * yValue;

        double xPower;
        double yPower;

        if(xValue < 0) {
            xPower = -xSquaredVal;
        } else {
            xPower = xSquaredVal;
        }

        if(yValue < 0) {
            yPower = -ySquaredVal;
        } else {
            yPower = ySquaredVal;
        }

        double leftPower = yPower + xPower;
        double rightPower = yPower - xPower;

        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        leftMotor.setPower(leftPower);

        rightMotor.setPower(rightPower);


    }
}
