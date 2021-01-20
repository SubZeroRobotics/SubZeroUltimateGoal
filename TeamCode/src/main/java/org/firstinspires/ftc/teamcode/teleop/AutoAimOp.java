package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Device;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@Config
@TeleOp(name = "COMPTELE")
public class AutoAimOp extends LinearOpMode {
    //Shooter
    Shooter shooter;
    Linkage linkage;
    Servo flicker;
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
    //vars
    public  double up = 0.27;
    public  double down = 0.87;
    public  double in = .05;
    public  double out = .32;
    public static double flapAngle = .32;
    public static double shooterPower = 1;
    public static long flickerDelay = 300;
    //gamepad
    boolean toggleShooter = false;
    boolean prevA = false;
    boolean toggleLinkage = false;
    boolean prevB = false;
    boolean toggleClaw = false;
    boolean prevRightStickButton = false;
    boolean toggleAUTOTURN;
    boolean prevA2 = false;
    public boolean toggleSlowMode = false;
    boolean prevLeftStickButton = false;
    public double slowModeMultiplier = .45;

    public double shotCounter = 0;
    //timers
    public ElapsedTime elapsedTime = new ElapsedTime();
    public ElapsedTime elapsedTime2 = new ElapsedTime();
    public ElapsedTime elapsedTime3 = new ElapsedTime();
    public ElapsedTime flapTimer = new ElapsedTime();
    //auto aim

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        linkage = new Linkage(hardwareMap,shooter, elapsedTime, elapsedTime2, .7,.38,.3,.52 );
        flicker = hardwareMap.get(Servo.class, "pusher");
        angleFlap = hardwareMap.get(Servo.class, "flap");
        wobblemech = new Wobblemech(hardwareMap, elapsedTime3);
        intake = new Intake(hardwareMap);
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        //auto aim
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.getLocalizer().setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

        angleFlap.setPosition(flapAngle);
        ElapsedTime timer = new ElapsedTime();
        boolean servoMoving = false;
        timer.reset();
        //wait for start
        flicker.setPosition(.52);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            // Declare a drive direction
            // Pose representing desired x, y, and angular velocity
            Pose2d driveDirection = new Pose2d();
            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

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



                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y ,
                                        -gamepad1.left_stick_x,
                                        -gamepad1.right_stick_x
                                )
                        );



                    //----------------------------------------------------------------------------------------
                    //auto aim
                    //----------------------------------------------------------------------------------------
                    //flicking
                    if (gamepad1.y && timer.milliseconds() >= 150 && !servoMoving) {
                        flicker.setPosition(.3);
                        servoMoving = true;
                        timer.reset();
                    }

                    if (timer.milliseconds() >= 150 && servoMoving) {
                        flicker.setPosition(.52);
                        servoMoving = false;
                        timer.reset();
                    }






                    //wobble
                    if (gamepad2.dpad_up) {
                        wobblemech.extend();
                    }
                    if (gamepad2.dpad_down) {
                        wobblemech.teleOpidle();
                    }

                    if (gamepad2.y) {
                        wobblemech.vertical();
                    }
                    //  ------------------------------------------------------------------
                    //--------------------------------------------------------------------
                    boolean aval2 = gamepad2.a;
                    if (aval2 && !prevA2) {
                        toggleClaw = !toggleClaw;
                    }

                    if (toggleClaw) {
                        wobblemech.grip();
                    } else {
                        wobblemech.letGo();
                    }
                    prevA2 = aval2;
                    //--------------------------------------------------------------------
                    if (gamepad2.x) {
                        angleFlap.setPosition(.5);
                        sleep(100);
                        angleFlap.setPosition(.38);
                        sleep(100);
                    }
                    if (gamepad2.b) {
                        angleFlap.setPosition(.5);
                        sleep(120);
                        angleFlap.setPosition(.32);
                        sleep(120);
                    }


            drive.getLocalizer().update();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

    }
    public void actuateFlicker() {
        linkage.flickerIn();
        sleep(150);
        linkage.flickerOut();
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