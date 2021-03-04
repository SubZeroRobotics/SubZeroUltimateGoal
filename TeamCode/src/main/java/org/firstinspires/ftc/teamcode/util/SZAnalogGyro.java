package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SZAnalogGyro {
    HardwareMap hw;
    public AnalogInput analogInput;
    public SZAnalogGyro(HardwareMap hw, String device_name){
        this.hw = hw;
        analogInput = hw.analogInput.get(device_name);
    }

    double getHeading(){
        double voltage = analogInput.getVoltage();
        double maxVoltage = analogInput.getMaxVoltage();
        return (voltage/maxVoltage) * 360;
    }
}
