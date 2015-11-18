package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class zDemoTank extends OpMode {

    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;

    @Override
    public void init() {

        leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front");

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

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

        rightFrontMotor.setPower(rightPower);
       
    }
}
