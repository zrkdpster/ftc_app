package com.qualcomm.ftcrobotcontroller.opmodes;

//import com.qualcomm.ftcrobotcontroller.AdafruitIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


 //Created by robot on 12/11/2015. *****************************************************************
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

     public int SRlock = 1;
     public int TRlock = 1;
     public int SRunlock = 0;
     public int TRunlock = 0;
    public int unlockincrement = 5;
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
        // need to set lock state of ratchets here**********************************************
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
        tapeMotor.setPower(tapeMaxPower);
        shoulderMotor.setPower(shoulderMaxPower);
        double shoulder = gamepad2.left_stick_y;
        double tape = gamepad2.right_stick_y;

        if (shoulder > Positive_Dead_Zone){
            shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition() + increment);
            telemetry.addData("shoulder : ", "Shoulder up   " + System.out.format("%.2f",shoulderMotorEncodercurrent));
        } else if (shoulder < Negative_Dead_Zone){
            if (shoulderMotor.getCurrentPosition() < Hard_Stop) {
                // don't go backwards beyond starting position
            } else {
                shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition() - increment);
                telemetry.addData("shoulder : ", "Shoulder down " + System.out.format("%.2f",shoulderMotorEncodercurrent));
            }
        }

        if (tape > Positive_Dead_Zone){
            tapeMotor.setTargetPosition(tapeMotor.getCurrentPosition() + increment);
            telemetry.addData("tape : ", "tape up   " + System.out.format("%.2f",tapeMotorEncodercurrent));
        } else if (tape < Negative_Dead_Zone) {
            if (tapeMotor.getCurrentPosition() < Hard_Stop) {
            } else {
                tapeMotor.setTargetPosition(tapeMotor.getCurrentPosition() - increment);
                telemetry.addData("tape : ", "tape down "+ System.out.format("%.2f",tapeMotorEncodercurrent));
            }
        }


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
         telemetry.addData("noodle : ", "Red noodle down");
         redNoodleServo.setPosition(0.0);

     } else {
         telemetry.addData("noodle : ", "Red noodle up");
         redNoodleServo.setPosition(1.0);
     }

    if (blueCount == Math.floor(blueCount / 2)*2) {
         telemetry.addData("noodle : ", " Blue noodle down");
         blueNoodleServo.setPosition(1.0);
     } else {
        telemetry.addData("noodle : ", " Blue noodle up");
        blueNoodleServo.setPosition(0.0);
     }

     // ratchet control goes here *************************************************
     /*
      lock
      if the a/x button is pressed check the state
      if state locked is true return
      if the state locked is false move the servo into the locked position

      unlock
      when the b/y button is pressed check the state
      if the state is unlocked return
      if the state is locked run the motor back a couple of ticks move the servo into the unlocked position
     */
     // lock servos
     if (gamepad2.a){
         if (shoulderRatchetLocked == true){

         }
          else {
             shoulderRatchet.setPosition(SRlock);
         }
       }

     if (gamepad2.x){
         if (tapeRatchetLocked == true){

         }
         else {
             tapeRatchet.setPosition(TRlock);
         }
     }
    // unlock servos
     if (gamepad2.b){
         if (shoulderRatchetLocked == false){

         }
         else {
             shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition() - unlockincrement );
             shoulderRatchet.setPosition(SRunlock);
         }
     }
     if (gamepad2.y){
         if (tapeRatchetLocked == false){

         }
         else {
             tapeMotor.setTargetPosition(tapeMotor.getCurrentPosition() - unlockincrement );
             tapeRatchet.setPosition(TRunlock);
         }
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
