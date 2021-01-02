package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;
import org.firstinspires.ftc.teamcode.util.TrajectoryStorage;

@Config
@TeleOp(name = "TELEOPEXP")
public class TELEOPEXP extends LinearOpMode {

    //Shooter
    public Shooter shooter;
    public Linkage linkage;
    Wobblemech wobblemech;
    Servo angleFlap;

    //Intake
    Intake intake;

    //non changing variables
    public  double flapAngle = .425;
    public  long flickerDelay = 145;
    public boolean bPressed = false;
    public boolean aPressed = false;
    public boolean rStickButton = false;
    public boolean aPressed2 = false;
    public boolean slowMode = false;
    public double multiplier = .45;
    public double forwardPower;
    public double reversePower;


    //gamepad
    boolean toggleShooter = false;
    boolean prevA = false;

    boolean toggleLinkage = false;
    boolean prevB = false;

    boolean toggleClaw = false;
    boolean prevB = false;





    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        linkage = new Linkage(hardwareMap, .7, .38, .3, 0.52);
        angleFlap = hardwareMap.get(Servo.class, "flap");
        intake = new Intake(hardwareMap);
        wobblemech = new Wobblemech(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            forwardPower = -gamepad1.left_trigger;
            reversePower = -gamepad1.right_trigger;
            intake.setPower(forwardPower - reversePower);
            //angle flap
            angleFlap.setPosition(flapAngle);
            //shooter
//            if (gamepad1.a) {
//                aPressed = !aPressed;
//                while (gamepad1.a);
//            }
//            if (aPressed) {
//                shooter.setNoPIDPower(.96);
//            } else {
//                shooter.setNoPIDPower(0);
//            }
            //------------------------------------------------------
            boolean aValue = gamepad1.a;
            if (aValue && !prevA) {
                toggleShooter = !toggleShooter;
            }

            if (toggleShooter) {
                shooter.setNoPIDPower(.96);
            } else {
                shooter.setNoPIDPower(.25);
            }
           prevA = aValue;

//------------------------------------------------------------------

            //linkage
//            if (gamepad1.b) {
//                bPressed = !bPressed;
//                while(gamepad1.b);
//            }
//            if (bPressed) {
//                linkage.raise();
//            } else {
//                linkage.lower();
//            }
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

            //drivetrain


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            if (gamepad1.x) {
                actuateFlicker();
                sleep(flickerDelay);
                actuateFlicker();
                sleep(flickerDelay);
                actuateFlicker();
                sleep(flickerDelay);
                actuateFlicker();
                sleep(flickerDelay);
                linkage.lower();
                sleep(250);
                toggleLinkage = !toggleLinkage;
            }

            if(gamepad2.dpad_up){
                wobblemech.extend();
            }
            if(gamepad2.dpad_down){
                wobblemech.teleOpidle();
            }

            if (gamepad2.a) {
                aPressed2 = !aPressed2;
                while (gamepad2.a);
            }

            if(aPressed2){
                wobblemech.letGo();
                sleep(100);
            }else {
                wobblemech.grip();
                sleep(100);
            }



            //slow mode
            if (gamepad1.right_stick_button) {
                rStickButton =! rStickButton;
                while (gamepad1.right_stick_button);
            }

            if(rStickButton){
                slowMode = true;
            }else {
                slowMode = false;
            }

        }
    }
    public void actuateFlicker() {
        linkage.flickerIn();
        sleep(flickerDelay);
        linkage.flickerOut();
    }
}