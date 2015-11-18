package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class jjkdDemoArcade extends OpMode {

    final double UP_POSITION = 0.0;
    final double DOWN_POSITION = 1.0;

    DcMotor leftFrontMotor;
    DcMotor leftRearMotor;
    DcMotor rightFrontMotor;
    DcMotor rightRearMotor;
    DcMotor spinnerMotor;
    Servo gripper;

    @Override
    public void init() {

        leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        leftRearMotor = hardwareMap.dcMotor.get("left_rear");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front");
        rightRearMotor = hardwareMap.dcMotor.get("right_rear");
        spinnerMotor = hardwareMap.dcMotor.get("spinner");

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        gripper.setPosition(UP_POSITION);
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

        leftFrontMotor.setPower(leftPower);
        leftRearMotor.setPower(leftPower);

        rightFrontMotor.setPower(rightPower);
        rightRearMotor.setPower(rightPower);

        // operate spinner motors, use gamepad buttons y and b

        if(gamepad1.y) {
            spinnerMotor.setPower(0.1);
        } else if(gamepad1.b) {
            spinnerMotor.setPower(-0.1);
        } else {
            spinnerMotor.setPower(0);
        }

        // control the gripper

        if(gamepad1.x) {
            gripper.setPosition(UP_POSITION);
        }
        if(gamepad1.a) {
            gripper.setPosition(DOWN_POSITION);
        }
    }
}
