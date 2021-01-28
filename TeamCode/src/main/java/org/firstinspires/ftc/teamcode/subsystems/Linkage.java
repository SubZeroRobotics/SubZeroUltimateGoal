package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Linkage {
    HardwareMap hardwareMap;
    Shooter shooter;
    Servo linkage;
    Servo flicker;


    public double flickerPush;
    public double flickerPull;
    public double linkageUp;
    public double linkageDown;
    public boolean FIRST_SHOT = false, SECOND_SHOT = false, THIRD_SHOT = false;

    private ElapsedTime fTimer = new ElapsedTime();
    private ElapsedTime flick3Timer = new ElapsedTime();

    public Linkage(HardwareMap hardwareMap, Shooter shooter, ElapsedTime elapsedTime, ElapsedTime elapsedTime2, double up, double down, double in, double out) {
        this.hardwareMap = hardwareMap;
        this.shooter = shooter;
        flicker = hardwareMap.get(Servo.class, "pusher");
        linkage = hardwareMap.get(Servo.class, "linkage");
        linkageUp = up;
        linkageDown = down;
        flickerPush = in;
        flickerPull = out;
        this.fTimer = elapsedTime;
        this.flick3Timer = elapsedTime2;
    }

    //mag
    public void raise() {
        linkage.setPosition(linkageUp);
    }

    public void lower() {
        linkage.setPosition(linkageDown);
    }

    //flicker
    public void flickerIn() {
        flicker.setPosition(flickerPush);
    }

    public void flickerOut() {
        flicker.setPosition(flickerPull);
    }

    //flicker automation



    public void flick() {
        fTimer.reset();
        flickerIn();
        if (fTimer.milliseconds() > 150) {
            flickerOut();
            fTimer.reset();
        }
    }



}
