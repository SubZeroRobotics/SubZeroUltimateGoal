package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.testing.VelocityPIDFController;


public class Shooter {
    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(.002, 0, 0);

    public static double RPM = 6000;

    // Copy your feedforward gains here
    public static double kV = .00051;
    public static double kA = .000255;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    public final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    public DcMotorEx myMotor1;
    public DcMotorEx myMotor2;

    public Shooter(HardwareMap hwmp){
         myMotor1 = hwmp.get(DcMotorEx.class, "sm1");
         myMotor2 = hwmp.get(DcMotorEx.class, "sm2");
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        myMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        for (LynxModule module : hwmp.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }



    public void setNoPIDPower(double power){
//        motor1.setPower(power);
//        motor2.setPower(power);
    }

    public void motorSetPIDpower(double decimal){

        double targetVelo = decimal * RPM * 28.0/ 1.5 / 60.0;

        // Call necessary controller methods
        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();

        lastTargetVelo = targetVelo;

        // Get the velocity from the motor with the encoder
        double motorPos = myMotor1.getCurrentPosition();
        double motorVelo = myMotor1.getVelocity();

        // Update the controller and set the power for each motor
        double power = veloController.update(motorPos, motorVelo);
        myMotor1.setPower(power);
        myMotor2.setPower(power);
    }
}
