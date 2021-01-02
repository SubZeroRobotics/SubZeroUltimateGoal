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
        gripper = hw.get(Servo.class, "grabber");
        arm = hw.get(Servo.class, "arm");
    }


    public void retract(){ arm.setPosition(.85); }

    public void extend(){
        arm.setPosition(.27);
    }

    public void grip(){
        gripper.setPosition(.1);
    }

    public void letGo(){
        gripper.setPosition(.4);
    }

    public void idle(){ arm.setPosition(.4);}
    public void teleOpidle(){ arm.setPosition(.85);}
    public void vertical(){ arm.setPosition(.625);}

    public void dropWobble(){
        extend();
        letGo();
        retract();
    }

    public void gripWobble(){
        letGo();
        extend();
        grip();
        retract();
    }

}
