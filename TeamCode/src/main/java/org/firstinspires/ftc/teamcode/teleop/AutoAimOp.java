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

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.GamepadProcessing;

@Config
@TeleOp(name = "AutoAimOp")
public class AutoAimOp extends LinearOpMode {
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
    //vars
    public  double up = 0.27;
    public  double down = 0.87;
    public  double in = .05;
    public  double out = .32;
    public static double flapAngle = .425;
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
    //timers
    public ElapsedTime elapsedTime = new ElapsedTime();
    public ElapsedTime elapsedTime2 = new ElapsedTime();
    public ElapsedTime elapsedTime3 = new ElapsedTime();
    public ElapsedTime flapTimer = new ElapsedTime();
    //auto aim
    public static double DRAWING_TARGET_RADIUS = 2;
    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }
    private Mode currentMode = Mode.NORMAL_CONTROL;
    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    private Vector2d targetPosition = new Vector2d(0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        linkage = new Linkage(hardwareMap,shooter, elapsedTime, elapsedTime2, .7,.38,.3,.52 );
        pusher = hardwareMap.get(Servo.class, "pusher");
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
        headingController.setInputBounds(-Math.PI, Math.PI);

        angleFlap.setPosition(flapAngle);
        //wait for start
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            // Declare a drive direction
            // Pose representing desired x, y, and angular velocity
            Pose2d driveDirection = new Pose2d();
            telemetry.addData("mode", currentMode);
            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            switch (currentMode) {
                case NORMAL_CONTROL:
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
                    boolean LstcickBval = gamepad1.left_stick_button;
                    if (LstcickBval && !prevLeftStickButton) {
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

                    prevLeftStickButton = LstcickBval;
                    //----------------------------------------------------------------------------------------
                    //auto aim
                    boolean rStickVBval = gamepad1.right_stick_button;
                    if(rStickVBval && !prevLeftStickButton){
                        toggleAUTOTURN = ! toggleAUTOTURN;
                    }
                    if(toggleAUTOTURN){
                        currentMode = Mode.ALIGN_TO_POINT;
                    }else{
                        currentMode = Mode.NORMAL_CONTROL;
                    }
                    prevLeftStickButton = rStickVBval;
                    //----------------------------------------------------------------------------------------
                    //flicking
                    if (gamepad1.x) {
                        linkage.flick3();
                        toggleLinkage = !toggleLinkage;
                    }
                    if (gamepad1.y) {
                        linkage.flick();
                    }
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
                    //AUTOMATED DROP
                    if (gamepad2.right_stick_button) {
                        wobblemech.dropWobble();
                    }
                    //CLAW
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
                        angleFlap.setPosition(.435);
                        sleep(100);
                    }
                    if (gamepad2.b) {
                        angleFlap.setPosition(.5);
                        sleep(100);
                        angleFlap.setPosition(.425);
                        sleep(100);
                    }
                    break;
                case ALIGN_TO_POINT:
                    // Switch back into normal driver control mode if joysticks are moving
                    if(gamepad1.left_stick_x > 0 || gamepad1.left_stick_y > 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_y > 0){
                        currentMode = Mode.NORMAL_CONTROL;
                    }


                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta = difference.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    // Draw the target on the field
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

                    // Draw lines to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                    break;

            }
            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
            headingController.update(poseEstimate.getHeading());

            // Update he localizer
            drive.getLocalizer().update();

            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Print pose to telemetry
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