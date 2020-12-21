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
import org.firstinspires.ftc.teamcode.util.TrajectoryStorage;

@Config
@TeleOp(name = "TeleOp")
public class FullTeleOp extends LinearOpMode {

    //Shooter
    public Shooter shooter;
    public Linkage linkage;
    Servo angleFlap;

    //Intake
    Intake intake;

    //non changing variables
    public static double flapAngle = .35;
    public static long flickerDelay = 145
            ;
    public boolean bPressed = false;
    public boolean aPressed = false;
    public boolean slowMode = false;
    public double forwardPower;
    public double reversePower;
    public Vector2d targetVector = new Vector2d(0,0);
    public double targetHeading = 0;

    //wobble mech

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    Mode currentMode = Mode.DRIVER_CONTROL;



    @Override
    public void runOpMode() throws InterruptedException {

        shooter = new Shooter(hardwareMap);
        linkage = new Linkage(hardwareMap, 0.27, 0.87, 0.12, 0.32);
        angleFlap = hardwareMap.get(Servo.class, "flap");
        intake = new Intake(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        drive.setPoseEstimate(TrajectoryStorage.getPose());
        drive.setPoseEstimate(new Pose2d(0,0,0));


        waitForStart();
        while (!isStopRequested()) {
            drive.update();
            //intake

            switch(currentMode) {
                case DRIVER_CONTROL:
                forwardPower = -gamepad1.left_trigger;
                reversePower = -gamepad1.right_trigger;
                intake.setPower(forwardPower - reversePower);
                //angle flap
                angleFlap.setPosition(flapAngle);
                //shooter
                if (gamepad1.a) {
                    aPressed = !aPressed;
                    while (gamepad1.a) ;
                }
                if (aPressed) {
                    shooter.setNoPIDPower(.96);
                } else {
                    shooter.setNoPIDPower(0);
                }
                //linkage
                if (gamepad1.b) {
                    bPressed = !bPressed;
                    while (gamepad1.b) ;
                }
                if (bPressed) {
                    linkage.raise();
                } else {
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


                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("CURRENT MODE", currentMode);
                telemetry.update();

                if (gamepad1.x) {
                    slowMode = false;
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
                    bPressed = !bPressed;
                }
                    if(gamepad2.a) {
                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetVector, targetHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                }
                    break;
                case AUTOMATIC_CONTROL:
//                    if (Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0) {
//                        drive.cancelFollowing();
//                        currentMode = Mode.DRIVER_CONTROL;
//                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
    public void actuateFlicker() {
        linkage.flickerIn();
        sleep(flickerDelay);
        linkage.flickerOut();
    }

}