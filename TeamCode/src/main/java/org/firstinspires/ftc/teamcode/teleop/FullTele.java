//package org.firstinspires.ftc.teamcode.teleop;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.Linkage;
//import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;
//
//@Config
//@TeleOp(name = "TELEOP")
//public class FullTele extends LinearOpMode {
//
//    //Shooter
//    DcMotor sm1;
//    DcMotor sm2;
//
//    Linkage linkage;
//    Servo pusher;
//    Servo angleFlap;
//    //Intake
//    DcMotor im1;
//    DcMotor im2;
//
//    //wobble
//    Wobblemech wobblemech;
//
//    //Dt
//
//    DcMotor FL;
//    DcMotor BL;
//    DcMotor FR;
//    DcMotor BR;
//
//    SampleMecanumDrive drive;
//
//
//    //vars
//    public  double up = 0.27;
//    public  double down = 0.87;
//
//    public  double in = .05;
//    public  double out = .32;
//
//    public static double flapAngle = .425;
//
//    public static double shooterPower = .96;
//
//    public static long flickerDelay = 150;
//
//    public boolean bPressed = false;
//
//    public boolean aPressed = false;
//
//    public boolean aPressed2 = false;
//
//    public boolean bPressed2 = false;
//
//
//    public boolean rightStickButtonPressed = false;
//
//    public boolean slowMode = false;
//
//    public double slowModeMultiplier = .45;
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        sm1 = hardwareMap.get(DcMotor.class, "sm1");
//        sm2 = hardwareMap.get(DcMotor.class, "sm2");
//        linkage = new Linkage(hardwareMap, .7, .38, .3, 0.52);
//        pusher = hardwareMap.get(Servo.class, "pusher");
//        angleFlap = hardwareMap.get(Servo.class, "flap");
//        wobblemech = new Wobblemech(hardwareMap);
//        drive = new SampleMecanumDrive(hardwareMap);
//        im1 = hardwareMap.get(DcMotor.class, "im1");
//        im2 = hardwareMap.get(DcMotor.class, "im2");
//        FL = hardwareMap.get(DcMotor.class, "FL");
//        BL = hardwareMap.get(DcMotor.class, "BL");
//        FR = hardwareMap.get(DcMotor.class, "FR");
//        BR = hardwareMap.get(DcMotor.class, "BR");
//
//        GamepadProcessing gamepad = new GamepadProcessing(gamepad1);
//
//
//
//        MotorConfigurationType flywheel1Config = sm1.getMotorType().clone();
//        MotorConfigurationType flywheel2Config = sm2.getMotorType().clone();
//
//        flywheel1Config.setAchieveableMaxRPMFraction(1.0);
//
//        sm1.setMotorType(flywheel1Config);
//
//        flywheel2Config.setAchieveableMaxRPMFraction(1.0);
//
//        sm2.setMotorType(flywheel2Config);
//
//        im1.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        FL.setDirection(DcMotorSimple.Direction.REVERSE);
//        BL.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        sm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        sm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        sm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        angleFlap.setPosition(flapAngle);
//
//
//        waitForStart();
//
//        while (opModeIsActive()){
//
//
//
//            //intake
//            double fowardPower = -gamepad1.left_trigger;
//            double reversePower = -gamepad1.right_trigger;
//
//            im1.setPower(fowardPower - reversePower);
//            im2.setPower(fowardPower - reversePower);
//
//            //shooter
//            if (gamepad1.a) {
//                aPressed = !aPressed;
//                while (gamepad1.a);
//            }
//
//            if(aPressed){
//                sm1.setPower(shooterPower);
//                sm2.setPower(shooterPower);
//            }else {
//                sm1.setPower(0);
//                sm2.setPower(0);
//            }
//            //linkage
//            if (gamepad1.b) {
//                bPressed = !bPressed;
//                while (gamepad1.b);
//            }
//
//            if(bPressed){
//                linkage.raise();
//            }else {
//                linkage.lower();
//            }
//
//            //slow mode yeeyee
//            if (gamepad1.right_stick_button) {
//                rightStickButtonPressed =! rightStickButtonPressed;
//                while (gamepad1.right_stick_button);
//            }
//
////            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
////            double x = gamepad1.left_stick_x;
////            double rx = gamepad1.right_stick_x;
//
//            if(rightStickButtonPressed){
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                -gamepad1.left_stick_y * slowModeMultiplier,
//                                -gamepad1.left_stick_x * slowModeMultiplier,
//                                -gamepad1.right_stick_x * slowModeMultiplier
//                        )
//                );
//            }else {
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                -gamepad1.left_stick_y,
//                                -gamepad1.left_stick_x,
//                                -gamepad1.right_stick_x
//                        )
//                );
//            }
//
//
//
//            //shoot 3
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
//            //shoot 1
//            if (gamepad1.y){
//                actuateFlicker();
//                sleep(flickerDelay);
//            }
//
//            //wobble yee yee
//
//            if(gamepad2.dpad_up){
//                wobblemech.extend();
//            }
//            if(gamepad2.dpad_down){
//                wobblemech.teleOpidle();
//            }
//            if(gamepad2.y){
//                wobblemech.vertical();
//            }
//
//            if (gamepad2.a) {
//                wobblemech.grip();
//                sleep(100);
//            }
//
//            if(gamepad2.b){
//                wobblemech.letGo();
//                sleep(100);
//            }
//
//            if(gamepad2.x){
//                angleFlap.setPosition(.5);
//                sleep(100);
//                angleFlap.setPosition(.435);
//                sleep(100);
//            }
//
//            if(isStopRequested()) return;
//
//
//
//
//
//        }
//
//
//
//    }
//
//    public void actuateFlicker() {
//        linkage.flickerIn();
//        sleep(150);
//        linkage.flickerOut();
//    }
//}