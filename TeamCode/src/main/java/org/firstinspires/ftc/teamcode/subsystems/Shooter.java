package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;


public class Shooter {
    //motor members
    public  DcMotorEx motor1, motor2;
    private  HardwareMap hw;

    //other constants
    static final double COUNTS_PER_REV = 28;
    private static final double ROTATIONS_PER_MINUTE = 5480;
    public static double currentRPM;
    private static long lastTime;
    private static double lastPosition;
    private boolean runningPID = false;
    public double currentTPS;

    //pid and elapsed time

    private static ElapsedTime linkageTimer = new ElapsedTime();
    private static ElapsedTime pusherTimer = new ElapsedTime();
    private static ElapsedTime shooterTimer = new ElapsedTime();

    public Shooter(HardwareMap hwmp){
        this.hw = hwmp;
       // linkage =  new SimpleServo(hw, "linkage");
     //   indexer = new SimpleServo(hw, "indexer");
        motor1 = hw.get(DcMotorEx.class, "sm1");
        motor2 = hw.get(DcMotorEx.class, "sm2");
        MotorConfigurationType flywheel1Config = motor1.getMotorType().clone();
        MotorConfigurationType flywheel2Config = motor2.getMotorType().clone();



        flywheel1Config.setAchieveableMaxRPMFraction(1.0);

        motor1.setMotorType(flywheel1Config);

        flywheel2Config.setAchieveableMaxRPMFraction(1.0);

        motor2.setMotorType(flywheel2Config);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastTime = System.currentTimeMillis();
        lastPosition = motor1.getCurrentPosition();
    }

    public void start(){
        runningPID = true;
    }

    public void stop(){
        motor2.setPower(0); motor1.setPower(0);
    }

    public void update(){
//        double currentPosition = motor1.getCurrentPosition();
//        long currentTime = System.currentTimeMillis();
//        double deltaRotations = (currentPosition - lastPosition) / COUNTS_PER_REV;
//        double deltaMinutes = (currentTime - lastTime) / 60000.0;
//        currentRPM = (deltaRotations) / (deltaMinutes);
//        lastTime = currentTime;
//        lastPosition = currentPosition;
//
//        if(runningPID){
//            double power =
//            motor1.setPower(power);
//            motor2.setPower(power);
//        }else{
//            stop();
//        }
    }


    public void setNoPIDPower(double power){
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void motorSetPIDpower(double power){
        double tps = ((ROTATIONS_PER_MINUTE / 60 / COUNTS_PER_REV) * power);
        PIDController pidcontroller = new PIDController(12,0,3, tps);
        currentTPS = motor1.getVelocity();
        double output = pidcontroller.calculate(currentTPS);
        motor1.setVelocity(output);
        motor2.setVelocity(output);
    }

    public double getCurrentVelocity(){
        return motor1.getVelocity();
    }

}
