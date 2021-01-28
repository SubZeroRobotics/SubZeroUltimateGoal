package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Intake {
    private static DcMotorEx motor1;
    private static DcMotorEx motor2;
    private static HardwareMap hw;

    public Intake(HardwareMap hwmp){
        hw = hwmp;
        motor1 = hw.get(DcMotorEx.class, "im1");
        motor2 = hw.get(DcMotorEx.class, "im2");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void setPower(double power){
        motor2.setPower(power * .8);
        motor1.setPower(power  * .8);
    }

    public void stop(){
        motor1.setPower(0);
        motor2.setPower(0);
    }
}
