  package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
import org.firstinspires.ftc.teamcode.testing.PathFollower;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.opencv.core.Mat;

@Config
@TeleOp(name = "COMPETITION TELEOP")
public class AutoPS extends LinearOpMode {
    //automation tings
    enum Mode {
        DRIVER_CONTROL,
        AUTO_PS
    }

    Mode currentMode = Mode.DRIVER_CONTROL;


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
    public static double flapAngle = .425;
    public double shooterPower = .94;
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

    public int shotCounter = 0;
    //timers
    public ElapsedTime elapsedTime = new ElapsedTime();
    public ElapsedTime elapsedTime2 = new ElapsedTime();
    public ElapsedTime elapsedTime3 = new ElapsedTime();
    public ElapsedTime flapTimer = new ElapsedTime();
    //auto aim
    Vector2d targetVector = new Vector2d(124,-25);


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
        PathFollower follower = new PathFollower(hardwareMap,drive);
        boolean autoTurn = false;
        follower.reset();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.getLocalizer().setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

        angleFlap.setPosition(flapAngle);
        ElapsedTime timer = new ElapsedTime();
        boolean servoMoving = false;
        timer.reset();
        //wait for start
        flicker.setPosition(.52);
        waitForStart();
        shooter.veloTimer.reset();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.update();
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
                        shooter.motorSetPIDpower(shooterPower);
                    } else {
                        shooter.motorSetPIDpower(0);
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
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );


                    if(gamepad1.left_bumper){
                        autoTurn = true;
                    }


                    boolean prevB = gamepad1.b;
                    boolean prevA = gamepad1.a;
                    boolean prevX = gamepad1.x;
                    boolean prevY = gamepad1.y;


                    while(autoTurn){
                        follower.turnToAbsolute(Math.toDegrees(Math.atan2( targetVector.getY() - drive.getPoseEstimate().getY(),targetVector.getX() - drive.getPoseEstimate().getX())), 1);

                        if(!gamepad1.atRest() || prevB != gamepad1.b || prevA != gamepad1.a || prevX != gamepad1.a || prevY != gamepad1.y){
                            drive.setWeightedDrivePower(new Pose2d(0,0,0));
                            autoTurn = false;
                            break;
                        }
                    }

                    if(gamepad1.dpad_down){
                        shooterPower = .675;
                    }


                    //----------------------------------------------------------------------------------------
                    //auto ps
                    if(gamepad1.y){
                        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
                        Trajectory goToRight = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> {
                                    //drop wobble
                                    linkage.raise();

                                })
                                .lineToConstantHeading(new Vector2d(-5,22))
                                .build();
                        angleFlap.setPosition(.52);
                        sleep(100);
                        angleFlap.setPosition(.44);
                        sleep(100);
                        drive.followTrajectory(goToRight);
                        actuateFlicker();

                        Trajectory goToMiddle = drive.trajectoryBuilder(goToRight.end())
                                .lineToConstantHeading(new Vector2d(-5,29))
                                .build();
                        drive.followTrajectory(goToMiddle);
                        actuateFlicker();

                        Trajectory goToLeft = drive.trajectoryBuilder(goToMiddle.end())
                                .lineToConstantHeading(new Vector2d(-5,37))

                                .build();
                        drive.followTrajectory(goToLeft);
                        actuateFlicker();
                            currentMode = Mode.AUTO_PS;
                    }

                    //----------------------------------------------------------------------------------------
                    //flicking
                    if (gamepad1.x && timer.milliseconds() >= 150 && !servoMoving) {
                        flicker.setPosition(.3);
                        servoMoving = true;
                        timer.reset();
                        shotCounter++;
                    }

                    if (timer.milliseconds() >= 150 && servoMoving) {
                        flicker.setPosition(.52);
                        servoMoving = false;
                        timer.reset();
                    }


                    if(shotCounter == 4 && linkage.isUp()) {
                        linkage.lower();
                    }else if(!linkage.isUp()){
                        shotCounter = 0;
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

                    telemetry.addData("mode", currentMode);
                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                    telemetry.addData("follower heading error", follower.headingError);
                    telemetry.update();
                    break;
                case AUTO_PS:
                    if (!drive.isBusy()) {
                        angleFlap.setPosition(.5);
                        sleep(120);
                        angleFlap.setPosition(.425);
                        sleep(120);
                        shooterPower = .94;
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;

            }

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