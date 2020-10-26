package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    //motor members
    private static DcMotorEx motor1, motor2;
    private static ServoEx linkage;
    private static ServoEx pusher;
    private static HardwareMap hw;

    //PID constants
    private static double kp = 0;
    private static double  ki = 0;
    private static double kd = 0;
    private static double kf = 0;

    //servo position varaibles
    private static double pusherTime = 0;
    private static double linkageTime = 0;
    private static double pusherPosition = 0;
    private static double linkagePosition = 0;


    private static PIDFController pidf = new PIDFController(kp,ki,kd,kf);
    private static ElapsedTime time = new ElapsedTime();


    public Shooter(HardwareMap hwmp){
        this.hw = hwmp;
        linkage =  hw.get(ServoEx.class, "linkage");
        pusher = hw.get(ServoEx.class, "pusher");

    }


    public static void actuateShootingSequence(){

    }

    //push ring command
    private static void pushRing(){
        time.reset();
        while(time.milliseconds() < pusherTime){
            pusher.setPosition(pusherPosition);
        }
    }

    //lift slides up

    private static void actuateLinkage(){
        time.reset();
        while(time.milliseconds() < linkageTime){
            linkage.setPosition(linkagePosition);
        }
    }


    //sets power to motor using PIDF controller
    private static void setPower(double power){
        
    }






}
