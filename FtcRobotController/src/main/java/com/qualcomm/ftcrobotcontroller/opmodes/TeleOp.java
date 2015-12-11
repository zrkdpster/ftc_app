package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.AdafruitIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by robot on 12/11/2015.
 */
public class TeleOp extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    Servo jjDumper;
    Servo redNoodle;
    Servo blueNoodle;

    boolean arcademode;

    AdafruitIMU boschBNO055;



    @Override
    public void init() {

        init_motors();
        init_servos();
        init_sensors();

    }

    public void init_motors() {
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        arcademode = true;
        telemetry.addData("Drive: ", "Arcade Mode");
    }

    public void init_servos() {

    }

    public void init_sensors() {

    }


    void handle_drivetrain() {

        if (arcademode) {    //  handle arcade mode here

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

        } else {            //  handle tank mode here
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

    public void handle_mode_commands() {

        if (gamepad1.left_bumper) {

            if (arcademode) {
                arcademode = false;
                telemetry.addData("Drive: ", "Tank Mode");
            } else {
                arcademode = true;
                telemetry.addData("Drive: ", "Arcade Mode");
            }

        }
    }

    @Override
    public void loop() {

        handle_drivetrain();
        handle_mode_commands();

    }
}
