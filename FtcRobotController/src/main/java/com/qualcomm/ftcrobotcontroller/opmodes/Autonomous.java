package com.qualcomm.ftcrobotcontroller.opmodes;

import android.os.SystemClock;

import com.qualcomm.ftcrobotcontroller.AdafruitIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by robot on 12/11/2015.
 */
public class Autonomous extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor elbowMotor;
    DcMotor shoulderMotor;


    Servo dumperServo;
    Servo redNoodleServo;
    Servo blueNoodleServo;

    boolean arcademode;

    AdafruitIMU boschBNO055;



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

    }

    public void init_sensors() {
       // long endOfRunTime = System.currentTimeMillis() + time;
    }
public void loop () {
}
}
