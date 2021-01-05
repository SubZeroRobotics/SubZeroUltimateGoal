package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;
import org.firstinspires.ftc.teamcode.util.GamepadProcessing;

@Config
@TeleOp(name = "TELEOPEX2")
public class TELEOPEX2 extends LinearOpMode {

    //Shooter
    Shooter shooter;

    Linkage linkage;
    Servo pusher;
    Servo angleFlap;
    //Intake
    Intake intake;

    //wobble
    Wobblemech wobblemech;

    //Dt

    DcMotor FL;
    DcMotor BL;
    DcMotor FR;
    DcMotor BR;

    SampleMecanumDrive drive;


    //vars
    public  double up = 0.27;
    public  double down = 0.87;

    public  double in = .05;
    public  double out = .32;

    public static double flapAngle = .425;

    public static double shooterPower = .96;

    public static long flickerDelay = 150;
    //gamepad
    boolean toggleShooter = false;
    boolean prevA = false;

    boolean toggleLinkage = false;
    boolean prevB = false;

    boolean toggleClaw = false;
    boolean prevA2 = false;

    public boolean toggleSlowMode = false;
    boolean prevRightStickButton = false;


    public double slowModeMultiplier = .45;

    public ElapsedTime Wtimer = new ElapsedTime();
    public ElapsedTime fTimer = new ElapsedTime();
    public ElapsedTime f3Timer = new ElapsedTime();
    public ElapsedTime flapTimer = new ElapsedTime();





    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        linkage = new Linkage(hardwareMap, .7, .38, .3, 0.52);
        pusher = hardwareMap.get(Servo.class, "pusher");
        angleFlap = hardwareMap.get(Servo.class, "flap");
        wobblemech = new Wobblemech(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");


        angleFlap.setPosition(flapAngle);


        waitForStart();

        while (opModeIsActive()){
            //intake
            //---------------------------------------------------------------------
            double forwardPower = -gamepad1.left_trigger;
            double reversePower = -gamepad1.right_trigger;
            intake.setPower(forwardPower - reversePower);

            //---------------------------------------------------------------------

            //SHOOTER
            //----------------------------------------------------------------------
            boolean aValue = gamepad1.a;
            if (aValue && !prevA) {
                toggleShooter = !toggleShooter;
            }

            if (toggleShooter) {
                shooter.setNoPIDPower(shooterPower);
            } else {
                shooter.setNoPIDPower(0);
            }
            prevA = aValue;
            //-------------------------------------------------------------------

            //--------------------------------------------------------------------
            boolean bValue = gamepad1.b;
            if (bValue && !prevB) {
                toggleLinkage = !toggleLinkage;
            }

            if (toggleLinkage) {
                linkage.raise();
            } else {
                linkage.lower();
            }
            prevB = bValue;
            //----------------------------------------------------------------------

            //slow mode
            //--------------------------------------------------------------------------
            boolean rstcickBval = gamepad1.right_stick_button;
            if ( rstcickBval&& !prevRightStickButton) {
                toggleSlowMode = !toggleSlowMode;
            }

            if (toggleSlowMode) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * slowModeMultiplier,
                                -gamepad1.left_stick_x * slowModeMultiplier,
                                -gamepad1.right_stick_x * slowModeMultiplier
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }
            prevRightStickButton = rstcickBval;
            //---------------------------------------------------------------------------------------



            //shoot 3
//            if (gamepad1.x){
//                actuateFlicker();
//                sleep(flickerDelay);
//                actuateFlicker();
//                sleep(flickerDelay);
//                actuateFlicker();
//                sleep(flickerDelay);
//                actuateFlicker();
//                sleep(flickerDelay);
//            }

            if(gamepad1.x){
                actuateFlicker();
                sleep(flickerDelay);
                actuateFlicker();
                sleep(flickerDelay);
                actuateFlicker();
                sleep(flickerDelay);
                actuateFlicker();
                sleep(flickerDelay);
                toggleLinkage =! toggleLinkage;
            }
            //shoot 1
            if (gamepad1.y){
                flick();
            }


            //wobble goal mech

            if(gamepad2.dpad_up){
                wobblemech.extend();
            }
            if(gamepad2.dpad_down){
                wobblemech.teleOpidle();
            }
//            if(gamepad2.y) {
//                wobblemech.vertical();
//            }

            if(gamepad2.y){
                wobblemech.vertical();
            }
            //  --------------------------------------------------------------------------------------------

            //CLAW
            //--------------------------------------------------------------------
            boolean aval2 = gamepad2.a;
            if ( aval2&& !prevA2) {
                toggleClaw = !toggleClaw;
            }

            if (toggleClaw) {
                wobblemech.grip();
            } else {
                wobblemech.letGo();
            }
            prevA2 = aval2;
            //----------------------------------------------------------------------

            if(gamepad2.x){
                angleFlap.setPosition(.5);
                sleep(100);
                angleFlap.setPosition(.435);
                sleep(100);
            }
            if(gamepad2.b){
                angleFlap.setPosition(.5);
                sleep(100);
                angleFlap.setPosition(.425);
                sleep(100);
            }




        }
    }
    public void actuateFlicker() {
        linkage.flickerIn();
        sleep(150);
        linkage.flickerOut();
    }

    public void callWobbleSequence() {
        Wtimer.reset();
        wobblemech.extend();
        if (Wtimer.milliseconds() >  300) {
            wobblemech.letGo();
            Wtimer.reset();
        }
        if (Wtimer.milliseconds() > 200) {
            wobblemech.retract();
            Wtimer.reset();
        }
    }

    public void flick3(){
        f3Timer.reset();
        linkage.flickerIn();
        if(fTimer.milliseconds() > 150){
            actuateFlicker();
            f3Timer.reset();
        }
        if(fTimer.milliseconds() > 150){
            actuateFlicker();
            f3Timer.reset();
        }
        if(fTimer.milliseconds() > 150){
            actuateFlicker();
            f3Timer.reset();
        }
    }

    public void powerShotFlap(){
        flapTimer.reset();
        angleFlap.setPosition(.5);
        if(flapTimer.milliseconds() > 100){
            angleFlap.setPosition(.435);
        }
    }

    public void highGoalFlap(){
        flapTimer.reset();
        angleFlap.setPosition(.5);
        if(flapTimer.milliseconds() > 100){
            angleFlap.setPosition(.425);
        }
    }


}