package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.BoschGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Aluminati on 9/21/2015.
 */
public class AluminatiImuCalibrate extends OpMode {
    private BoschGyro boschGyro;

    public AluminatiImuCalibrate() {
    }

    @Override
    public void init() {
        boschGyro = new BoschGyro(hardwareMap, "imu",true);
    }

    @Override
    public void init_loop() {

                telemetry.addData("1. Calibration",Integer.toHexString(boschGyro.calibrationStatus()));
                telemetry.addData("2. Sys Calibration",Integer.toHexString(boschGyro.sysCalStatus()));
                telemetry.addData("3. Calibration",Integer.toHexString(boschGyro.calibrationStatus()));
                telemetry.addData("4. Heading", boschGyro.getHeading());

    }

    // This is generally the "forever" loop in RobotC. But don't do a forever loop
    // The FTC App will constantly call this loop code.
    @Override
    public void loop() {
        // Calibration program - does nothing - not to be used for autonomous

        telemetry.addData("1. Calibration",Integer.toHexString(boschGyro.calibrationStatus()));
        telemetry.addData("2. Sys Calibration",Integer.toHexString(boschGyro.sysCalStatus()));
        telemetry.addData("3. Calibration",Integer.toHexString(boschGyro.calibrationStatus()));
        telemetry.addData("4. Heading", boschGyro.getHeading());

    }

    @Override
    public void stop() {

    }
}