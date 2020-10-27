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

    //servo position variables
    private static double pusherTime = 0;
    private static double pusherTime2 = 0;
    private static double linkageTime = 0;
    private static double linkageTime2 = 0;
    private static double pusherPosition = 0;
    private static double pusherPosition2 = 0;
    private static double linkagePosition = 0;
    private static double linkagePosition2 = 0;

    //other constants
    private static double COUNTS_PER_REV = 7.0;
    private static double ROTATIONS_PER_MINUTE = 5800.0;

    //pid and elapsed time
    private static PIDFController pidf = new PIDFController(kp,ki,kd,kf);
    private static ElapsedTime linkageTimer = new ElapsedTime();
    private static ElapsedTime pusherTimer = new ElapsedTime();
    private static ElapsedTime shooterTimer = new ElapsedTime();


    public Shooter(HardwareMap hwmp){
        this.hw = hwmp;
        linkage =  hw.get(ServoEx.class, "linkage");
        pusher = hw.get(ServoEx.class, "pusher");
        motor1 = hw.get(DcMotorEx.class, "motor1");
        motor2 = hw.get(DcMotorEx.class, "motor2");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    //shooting yeet
    public  void actuateShootingSequence(double power){
        //lift hopper up
        liftLinkage();
        //turn on shooter
        setPower(power);
        ///push ring
        shooterTimer.reset();
        //loop through and flick with a 250 milli wait
        for(int i = 0; i < 2; i++){
            while(shooterTimer.milliseconds() < 250) {
                pushRing();
            }
            shooterTimer.reset();
        }
        lowerLinkage();

    }

    //stop motor lol
    public static void stopFlyWheel(){ motor1.setPower(0.0);  motor2.setPower(0.0); }

    //push ring command
    private static void pushRing(){
        pusherTimer.reset();
        while(pusherTimer.milliseconds() < pusherTime){
            pusher.setPosition(pusherPosition);
        }
        pusherTimer.reset();
        while(pusherTimer.milliseconds() < pusherTime2){
            pusher.setPosition(pusherPosition);
        }
    }

    //lift slides up

    private static void liftLinkage(){
        linkageTimer.reset();
        while(linkageTimer.milliseconds() < linkageTime){
            linkage.setPosition(linkagePosition);
        }
    }

    //lower slides
    private static void lowerLinkage(){
        linkageTimer.reset();
        while(linkageTimer.milliseconds() < linkageTime){
            linkage.setPosition(linkagePosition2);
        }
    }

    //sets power to motor using PIDF controller
    private static void setPower(double power){
        power = power * ((ROTATIONS_PER_MINUTE / 60) * COUNTS_PER_REV);
        pidf.setSetPoint(power);
        double output = pidf.calculate(motor1.getVelocity());
        motor1.setVelocity(output);
        motor2.setVelocity(output);
    }


}
