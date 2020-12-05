package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


@TeleOp(name = "TeleOpOBJ")
public class TeleOpOBJ extends LinearOpMode {

    //Shooter
    DcMotor sm1;
    DcMotor sm2;

    Servo linkage;
    Servo pusher;
    Servo angleFlap;
    //Intake
    DcMotor im1;
    DcMotor im2;

    //Dt

    DcMotor FL;
    DcMotor BL;
    DcMotor FR;
    DcMotor BR;


    //vars
    public  double up = 0.27;
    public  double down = 0.87;

    public  double in = .05;
    public  double out = .32;

    public static double flapAngle = .35;

    public static double shooterPower = .95;

    public static long flickerDelay = 200;

    public boolean bPressed = false;

    public boolean aPressed = false;



    @Override
    public void runOpMode() throws InterruptedException {

        sm1 = hardwareMap.get(DcMotor.class, "sm1");
        sm2 = hardwareMap.get(DcMotor.class, "sm2");
        linkage = hardwareMap.get(Servo.class, "linkage");
        pusher = hardwareMap.get(Servo.class, "pusher");
        angleFlap = hardwareMap.get(Servo.class, "flap");
        im1 = hardwareMap.get(DcMotor.class, "im1");
        im2 = hardwareMap.get(DcMotor.class, "im2");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");



        MotorConfigurationType flywheel1Config = sm1.getMotorType().clone();
        MotorConfigurationType flywheel2Config = sm2.getMotorType().clone();

        flywheel1Config.setAchieveableMaxRPMFraction(1.0);

        sm1.setMotorType(flywheel1Config);

        flywheel2Config.setAchieveableMaxRPMFraction(1.0);

        sm2.setMotorType(flywheel2Config);

        im1.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        sm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        waitForStart();

        while (opModeIsActive()){

            //intake
            double fowardPower = -gamepad1.left_trigger;
            double reversePower = -gamepad1.right_trigger;

            im1.setPower(fowardPower - reversePower);
            im2.setPower(fowardPower - reversePower);

            angleFlap.setPosition(flapAngle);



            //shooter
            if (gamepad1.a) {
                aPressed = !aPressed;
                while (gamepad1.a);
            }

            if(aPressed){
                sm1.setPower(shooterPower);
                sm2.setPower(shooterPower);
            }else {
                sm1.setPower(0);
                sm2.setPower(0);
            }
            //linkage
            if (gamepad1.b) {
                bPressed = !bPressed;
                while (gamepad1.b);
            }

            if(bPressed){
                linkage.setPosition(up);
            }else {
                linkage.setPosition(down);
            }



            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            FL.setPower(y + x + rx);
            BL.setPower(y - x + rx);
            FR.setPower(y - x - rx);
            BR.setPower(y + x - rx);

            if (gamepad1.x){
                sleep(20);
                actuateFlicker();
                sleep(flickerDelay);
                actuateFlicker();
                sleep(flickerDelay);
                actuateFlicker();
                sleep(flickerDelay);
            }




        }
    }

    public void actuateFlicker(){
        pusher.setPosition(in);
        sleep(flickerDelay);
        pusher.setPosition(out);
    }
}