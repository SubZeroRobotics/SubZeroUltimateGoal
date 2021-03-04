package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SZMotor {
    DcMotorEx motor;
    HardwareMap hw;

    public SZMotor(HardwareMap hw, String device_name){
        this.hw = hw;
        motor = hw.get(DcMotorEx.class, device_name);
    }

    public void setPower(double power){

    }
}
