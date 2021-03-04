package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;

@TeleOp
@Config
public class PStest extends LinearOpMode {
    public Linkage linkage;
    public Shooter shooter;
    public Intake intake;
    public Wobblemech wobblemech;
    public Servo angleFlap;



    public static double shooterPower = .4;
    public static double flap = .44;



    double ringCount = 0;

    ElapsedTime timer = new ElapsedTime();

    boolean servoMoving = false;


    public ElapsedTime elapsedTime2 = new ElapsedTime();
    public ElapsedTime elapsedTime = new ElapsedTime();
    public ElapsedTime elapsedTime3 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkage = new Linkage(hardwareMap,shooter, elapsedTime, elapsedTime2, .7,.38,.28,.52 );
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobblemech = new Wobblemech(hardwareMap, elapsedTime3);
        angleFlap = hardwareMap.get(Servo.class, "flap");

        waitForStart();
        timer.reset();
        shooter.veloTimer.reset();
        while(opModeIsActive()){
            linkage.raise();
            shooter.motorSetPIDpower(shooterPower);
            angleFlap.setPosition(flap);

            if (gamepad1.x && timer.milliseconds() >= 150 && !servoMoving) {
                linkage.flick();
                servoMoving = true;
                timer.reset();
            }

            if (timer.milliseconds() >= 150 && servoMoving) {
                linkage.flickerOut();
                servoMoving = false;
                timer.reset();
            }

        }
    }
}
