package com.qualcomm.ftcrobotcontroller.opmodes;

//import com.qualcomm.ftcrobotcontroller.AdafruitIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by robot on 12/11/2015.
 */
public class TeleOp1 extends OpMode {
   // define motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor tapeMotor;
    DcMotor shoulderMotor;

//define servos
    Servo redNoodleServo;
    Servo blueNoodleServo;
    Servo shoulderRatchet;
    Servo tapeRatchet;

    public boolean arcademode;


    public int tapeMotorEncodercurrent;
    public int shoulderMotorEncodercurrent;

    public int blueCount;
    public int redCount;

    public boolean tapeRatchetLocked;
    public boolean shoulderRatchetLocked;

    //DcMotorController.RunMode tapeRunMode;
    //DcMotorController.RunMode shoulderRunMode;

    public double tankMaxPower          = 1.0;
    public double arcadeMaxPower        = 1.0;

    public double tapeMaxPower          = 0.1;
    public double shoulderMaxPower      = 0.1;

    public double maxXJoystick          = 1.0;

    public int increment = 15;
    public double Positive_Dead_Zone    = 0.05;
    public double Negative_Dead_Zone    = -0.05;
    public int Hard_Stop                = 5;

    @Override
    public void init() {
        arcademode = false;
        init_motors();
        init_servos();
        init_sensors();

    }


    /*
  * Code to run when the op mode is first enabled goes here
  * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
  */
    @Override
    public void start() {

        tapeMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        shoulderMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

    }
// function to choose what text to display when the drive mode is toggled
    public String drive_mode(boolean mode) {
        if(mode) {
            return("Arcade Mode");
        } else {
            return("Tank Mode");
        }
    }


//initialize drivetrain
    public void init_motors() {
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        arcademode = false;
        telemetry.addData("Drive: ", drive_mode(arcademode));
//initialize arm
        tapeMotor = hardwareMap.dcMotor.get("tape");
        shoulderMotor = hardwareMap.dcMotor.get("shoulder");


    }
//initialize servos
    public void init_servos() {
//
        redNoodleServo = hardwareMap.servo.get("redNoodle");
        blueNoodleServo = hardwareMap.servo.get("blueNoodle");
        shoulderRatchet = hardwareMap.servo.get("shoulderRatchet");
        tapeRatchet = hardwareMap.servo.get("tapeRatchet");

        // need to send servos to 'home' positions here*****************************************
    }

    public void init_sensors() {

    }


    void handle_drivetrain() {

        if (arcademode) {    //  code for arcade mode here

            telemetry.addData("Drive: ", drive_mode(arcademode));

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

        } else {            //  code for tank mode here

            telemetry.addData("Drive: ", drive_mode(arcademode));

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
// code that switches between arcade and tank modes
    public void handle_mode_commands() {

        if (gamepad1.left_bumper) {

            if (arcademode) {
                arcademode = false;
            } else {
                arcademode = true;
            }
            telemetry.addData("Drive: ", drive_mode(arcademode));
        }
    }
   // code to run the arm
    public void handle_arm() {

        double shoulder = gamepad2.left_stick_x;
        double tape = gamepad2.right_stick_x;


        if (shoulder > Positive_Dead_Zone){
            shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition() + increment);
            telemetry.addData("Controller 2 : ", "Shoulder up");
            telemetry.addData("Controller 2 : ", shoulderMotorEncodercurrent);
        } else if (shoulder < Negative_Dead_Zone){
            if (shoulderMotor.getCurrentPosition() < Hard_Stop) {
                // don't go backwards beyond starting position
            } else {
                shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition() - increment);
                telemetry.addData("Controller 2 : ", "Shoulder down");
                telemetry.addData("Controller 2 : ", shoulderMotorEncodercurrent);
            }
        }

        if (tape > Positive_Dead_Zone){
            tapeMotor.setTargetPosition(tapeMotor.getCurrentPosition() + increment);
            telemetry.addData("Controller 2 : ", "tape up");
            telemetry.addData("Controller 2 : ", tapeMotorEncodercurrent);
        } else if (tape < Negative_Dead_Zone) {
            if (tapeMotor.getCurrentPosition() < Hard_Stop) {
            } else {
                tapeMotor.setTargetPosition(tapeMotor.getCurrentPosition() - increment);
                telemetry.addData("Controller 2 : ", "tape down");
                telemetry.addData("Controller 2 : ", tapeMotorEncodercurrent);
            }
        }

        tapeMotor.setPower(tapeMaxPower);
        shoulderMotor.setPower(shoulderMaxPower);
    }
    // code to control all the various servos
 public void handle_servos() {

     if (gamepad2.right_bumper){
         ++redCount;
     }

     if (gamepad2.left_bumper){
         ++blueCount;
     }

     if (redCount == Math.floor(redCount / 2)*2) {
         telemetry.addData("Controller 2 : ", "Red noodle down");
         redNoodleServo.setPosition(0.0);

     } else {
         telemetry.addData("Controller 2 : ", "Red noodle up");
         redNoodleServo.setPosition(1.0);
     }

    if (blueCount == Math.floor(blueCount / 2)*2) {
         telemetry.addData("Controller 2 : ", " Blue noodle down");
         blueNoodleServo.setPosition(1.0);
     } else {
        telemetry.addData("Controller 2 : ", " Blue noodle up");
        blueNoodleServo.setPosition(0.0);
     }

     // ratchet control goes here *************************************************







 }


    @Override
    public void loop() {

        handle_drivetrain();
        handle_mode_commands();
        handle_servos();
        handle_arm();

    }
}
