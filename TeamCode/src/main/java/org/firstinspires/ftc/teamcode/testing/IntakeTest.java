package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeTest extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "intakemotor1");
        motor2 = hardwareMap.get(DcMotor.class, "intakemotor2");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){

            double fowardPower = gamepad1.left_trigger;
            double reversePower = gamepad1.right_trigger;

            motor1.setPower(fowardPower - reversePower);
            motor2.setPower(fowardPower - reversePower);
        }
    }
}
