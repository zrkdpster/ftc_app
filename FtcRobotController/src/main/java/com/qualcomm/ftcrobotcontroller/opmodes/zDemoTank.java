package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class zDemoTank extends OpMode {

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
