package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;


public class Shooter {
    //motor members
    public  DcMotorEx motor1, motor2;
    private  HardwareMap hw;

    //other constants
     static final double COUNTS_PER_REV = 28;
    private static final double ROTATIONS_PER_MINUTE = 5480;
    private static double currentRPM;
    private static long lastTime;
    private static double lastPosition;
    private boolean runningPID = false;
    
    //pid and elapsed time
    private PIDController pidcontroller;
    private static ElapsedTime linkageTimer = new ElapsedTime();
    private static ElapsedTime pusherTimer = new ElapsedTime();
    private static ElapsedTime shooterTimer = new ElapsedTime();

    public Shooter(HardwareMap hwmp){
        this.hw = hwmp;
       // linkage =  new SimpleServo(hw, "linkage");
     //   indexer = new SimpleServo(hw, "indexer");
        motor1 = hw.get(DcMotorEx.class, "motor1");
        motor2 = hw.get(DcMotorEx.class, "motor2");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastTime = System.currentTimeMillis();
        lastPosition = motor1.getCurrentPosition();
        pidcontroller = new PIDController(0.0,0.0,0.0,.9, 5480);
    }

    public void start(){
        runningPID = true;
    }

    public void stop(){
        runningPID = false;
    }

    public void update(){
        double currentPosition = motor1.getCurrentPosition();
        long currentTime = System.currentTimeMillis();
        double deltaRotations = (currentPosition - lastPosition) / COUNTS_PER_REV;
        double deltaMinutes = (currentTime - lastTime) / 60000.0;
        currentRPM = (deltaRotations) / (deltaMinutes);
        lastTime = currentTime;
        lastPosition = currentPosition;

        if(runningPID){
           double power =  pidcontroller.calculate(currentRPM);
            motor1.setPower(power);
            motor2.setPower(power);
        }

    }

    public void setNoPIDPower(double power){
        motor1.setPower(power);
        motor2.setPower(power);
    }
}
