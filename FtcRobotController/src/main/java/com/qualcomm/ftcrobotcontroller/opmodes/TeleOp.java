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
    DcMotor elbowMotor;
    DcMotor shoulderMotor;


    Servo dumperServo;
    Servo redNoodleServo;
    Servo blueNoodleServo;

    boolean arcademode;

    AdafruitIMU boschBNO055;

    public int elbowMotorEncodercurrent;
    public int shoulderMotorEncodercurrent;

    public double tankMaxPower      = 1.0;
    public double arcadeMaxPower    = 1.0;

    public double maxXJoystick      = 1.0;

    public int incremment = 5;

    @Override
    public void init() {

        init_motors();
        init_servos();
        init_sensors();

    }
//initialize drivetrain
    public void init_motors() {
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        arcademode = true;
        telemetry.addData("Drive: ", "Arcade Mode");
//initialize arm
        elbowMotor = hardwareMap.dcMotor.get("elbow");
        shoulderMotor = hardwareMap.dcMotor.get("shoulder");
        //xMotor.setDirection(DcMotor.Direction.REVERSE); ******might need

    }

    public void init_servos() {
        dumperServo = hardwareMap.servo.get("dumper");
        redNoodleServo = hardwareMap.servo.get("redNoodle");
        blueNoodleServo = hardwareMap.servo.get("blueNoodle");
    }

    public void init_sensors() {

    }


    void handle_drivetrain() {

        if (arcademode) {    //  handle arcade mode here

            telemetry.addData("Drive: ", "Arcade Mode");

            double xValue = -gamepad1.left_stick_x;
            double yValue = -gamepad1.left_stick_y;

            xValue = xValue * maxXJoystick;

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

            leftMotor.setPower(leftPower * arcadeMaxPower);

            rightMotor.setPower(rightPower * arcadeMaxPower);

        } else {            //  handle tank mode here

            telemetry.addData("Drive: ", "Tank Mode");

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

            leftMotor.setPower(leftPower * tankMaxPower);

            rightMotor.setPower(rightPower * tankMaxPower);

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
    public void handle_arm() {

        if (gamepad1.dpad_up){
            telemetry.addData("Controller: ", "dpad_up");
            elbowMotor.setTargetPosition(elbowMotor.getCurrentPosition() + incremment);
        }
        if (gamepad1.dpad_down){
            telemetry.addData("Controller: ", "dpad_down");
            elbowMotor.setTargetPosition(elbowMotor.getCurrentPosition() - incremment);
        }
        if (gamepad1.dpad_left){
            telemetry.addData("Controller: ", "dpad_left");
            shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition() + incremment);
        }
        if (gamepad1.dpad_right){
            telemetry.addData("Controller: ", "dpad_uright");
            shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition() - incremment);
        }
    }
 public void handle_servos() {
     if (gamepad1.a){
         telemetry.addData("Controller: ", "a");
         dumperServo.setPosition(1.0);
     } else {
         dumperServo.setPosition(0.0);
     }

     if (gamepad1.x){
         telemetry.addData("Controller: ", "x");
         redNoodleServo.setPosition(1.0);
     } else {
         redNoodleServo.setPosition(0.0);
     }

     if (gamepad1.y){
         telemetry.addData("Controller: ", "y");
         blueNoodleServo.setPosition(1.0);
     } else {
         blueNoodleServo.setPosition(0.0);
     }


 }


    @Override
    public void loop() {

        handle_drivetrain();
        handle_mode_commands();
        handle_servos();
        handle_arm();

    }
}
