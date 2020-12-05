package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp(name = "LinkageTesting")
public class LinkageTesting extends LinearOpMode {

    Servo linkage;
    Servo flicker;
    public static double up = 0.27;
    public static double down = 0.87;
    public static double in = 0;
    public static double out = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        linkage = hardwareMap.get(Servo.class, "linkage");
        flicker = hardwareMap.get(Servo.class, "pusher");
        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.left_trigger > .8) {
                //shooter.start();
                shooter.setNoPIDPower(1);
            }else{
                shooter.setNoPIDPower(0);
            }

            if(gamepad1.a){
                linkage.setPosition(up);
            }
            if(gamepad1.b){
                linkage.setPosition(down);
            }

            if(gamepad1.x){
                flicker.setPosition(in);
            }
            if(gamepad1.y){
                flicker.setPosition(out);

            }


        }
        }
    }

