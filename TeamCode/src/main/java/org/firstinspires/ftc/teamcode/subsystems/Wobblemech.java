package org.firstinspires.ftc.teamcode.subsystems;

import android.os.UserManager;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Wobblemech {
    HardwareMap hw;
    Servo gripper;
    Servo arm;
    ElapsedTime timer;
    public Wobblemech(HardwareMap hw, ElapsedTime timer){
        this.hw = hw;
        gripper = hw.get(Servo.class, "grabber");
        arm = hw.get(Servo.class, "arm");
        this.timer = timer;
    }


    public void retract(){ arm.setPosition(.85); }

    public void extend(){
        arm.setPosition(.27);
    }

    public void grip(){
        gripper.setPosition(.15);
    }

    public void letGo(){
        gripper.setPosition(.55);
    }

    public void idle(){ arm.setPosition(.4);}
    public void teleOpidle(){ arm.setPosition(.85);}
    public void vertical(){ arm.setPosition(.625);}

    public void dropWobble(){
        timer.reset();
        extend();
        if(timer.milliseconds() > 150){
            letGo();
            timer.reset();
        }
        if(timer.milliseconds() > 100){
            retract();
        }
    }



}
