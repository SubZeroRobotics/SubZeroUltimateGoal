package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TestOpMode extends OpMode {

    //declaring motor variables, setting them to nothing due to them not being hardware mapped yet
    DcMotor leftDriveMotor = null;
    DcMotor rightDriveMotor = null;
    //creating a variable of type double, this holds a value of 537.6 for the ticks counts of a motor
    Double  TICKS_PER_REV = 537.6;

    @Override
    public void init() {
        //mapping the motors to the hardware id's of "port1Motor", and "port2Motor"
        leftDriveMotor = hardwareMap.get(DcMotor.class, "port1Motor");
        rightDriveMotor = hardwareMap.get(DcMotor.class, "port2Motor");
    }

    @Override
    public void loop() {

    }
}
