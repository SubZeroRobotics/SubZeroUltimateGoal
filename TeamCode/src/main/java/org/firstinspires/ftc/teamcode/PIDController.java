package org.firstinspires.ftc.teamcode;

public class PIDController {
    private final double kp,ki,kd,setPoint;
    private double accumulatedError,lastError = 0;

    public PIDController(double kp, double ki, double kd, double setPoint){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.setPoint = setPoint;
    }


    public double calculate(double currentValue){
        double error = setPoint - currentValue;

        return kp * error;
    }
}
