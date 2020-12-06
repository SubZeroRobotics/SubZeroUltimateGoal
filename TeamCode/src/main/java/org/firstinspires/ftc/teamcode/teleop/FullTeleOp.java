package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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


@TeleOp(name = "FullTeleOp")
public class FullTeleOp extends LinearOpMode {

    //Shooter
    public Shooter shooter;
    public Linkage linkage;
    Servo angleFlap;
    Intake intake;

    //statics
    public static double flapAngle = .35;
    public static double shooterPower = .95;
    public static long flickerDelay = 200;
    public boolean bPressed = false;
    public boolean aPressed = false;
    public double forwardPower;
    public double reversePower;
    public static final double  goalX = 0;
    public static final double goalY = 0;
    public double angleToShoot;



    @Override
    public void runOpMode() throws InterruptedException {

        shooter = new Shooter(hardwareMap);
        linkage = new Linkage(hardwareMap, 0.27,0.87, 0.05,0.32);
        angleFlap = hardwareMap.get(Servo.class, "flap");
        intake = new Intake(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (!isStopRequested()){

            //intake
             forwardPower = -gamepad1.left_trigger;
             reversePower = -gamepad1.right_trigger;
             intake.setPower(forwardPower - reversePower);

            //angle flap
            angleFlap.setPosition(flapAngle);


            //shooter
            if (gamepad1.a) {
                aPressed = !aPressed;
                while (gamepad1.a);
            }
            if(aPressed){
                shooter.setNoPIDPower(shooterPower);
            }else {
                shooter.stop();
            }

            //linkage
            if (gamepad1.b) {
                bPressed = !bPressed;
                while (gamepad1.b);
            }
            if(bPressed){
                linkage.raise();
            }else {
                linkage.lower();
            }


            //drivetrain
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("angle to shoot", getAngleToShoot());
            telemetry.update();


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
        linkage.flickerIn();
        sleep(flickerDelay);
        linkage.flickerOut();
    }

    public void turnToGoal(Pose2d robotPose, SampleMecanumDrive drive){
         angleToShoot = Math.atan2(goalX - robotPose.getX(), goalY - robotPose.getY()) - robotPose.getHeading();
        drive.turnAsync(angleToShoot);
    }

    public double getAngleToShoot(){ return Math.toDegrees(angleToShoot);}

}