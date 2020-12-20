package org.firstinspires.ftc.teamcode.subsystems;

import android.os.UserManager;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobblemech {
    HardwareMap hw;
    Servo gripper;
    Servo arm;
    public Wobblemech(HardwareMap hw){
        this.hw = hw;
        gripper = hw.get(Servo.class, "gripper");
        arm = hw.get(Servo.class, "arm");
    }


    public void retract(){
        arm.setPosition(0);
    }

    public void extend(){
        arm.setPosition(1);
    }

    public void grip(){
        gripper.setPosition(0);
    }

    public void letGo(){
        gripper.setPosition(1);
    }

    public void dropWobble(){
        extend();
        grip();
        retract();
    }

}
