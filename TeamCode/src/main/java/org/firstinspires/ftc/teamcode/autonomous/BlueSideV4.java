package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.trajectories.AutoTrajectories;
import org.firstinspires.ftc.teamcode.autonomous.trajectories.AutoTrajectoriesHigh;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;
import org.firstinspires.ftc.teamcode.util.TrajectoryStorage;
import org.opencv.core.Mat;

import java.util.Arrays;


@Autonomous(name = "BlueSideV4")
public class BlueSideV4 extends LinearOpMode {

    public enum State {
        RING1,
        RING4,
        RING0,
        CONTINUE,
        IDLE;
    }


    //  State ring_position = State.RING4;

    public Linkage linkage;
    public Shooter shooter;
    public Intake intake;
    public Wobblemech wobblemech;
    public Servo angleFlap;
    public Pose2d startPose = new Pose2d(0, 0, 0);

    public static double rows = .12;
    public static double rectCols = .39;
    public static double rect2Cols = .46;
    double ringCount = 0;


    public ElapsedTime elapsedTime2 = new ElapsedTime();
    public ElapsedTime elapsedTime = new ElapsedTime();
    public ElapsedTime elapsedTime3 = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkage = new Linkage(hardwareMap,shooter, elapsedTime, elapsedTime2, .7,.38,.3,.52 );
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobblemech = new Wobblemech(hardwareMap, elapsedTime3);
        angleFlap = hardwareMap.get(Servo.class, "flap");




        RingDetectorV3 detector = new RingDetectorV3("BLUE",hardwareMap, telemetry,rows,rectCols,rect2Cols);
        detector.init();
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        State pathState = State.IDLE;


        while(!isStarted()) {
            ringCount = detector.getRingPosition();
            FtcDashboard.getInstance().startCameraStream(detector.phoneCam, 20);


            packet.put("Stack Height", ringCount);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Stack Height", ringCount);
            telemetry.addData("LowColor", detector.getVal());
            telemetry.addData("UpColor", detector.getVal2());
            telemetry.update();
        }

        if(ringCount == 4.0){
            pathState = State.RING4;
        }else if(ringCount == 1.0){
            pathState = State.RING1;
        }else{
            pathState = State.RING0;
        }





        //vision code here
        waitForStart();

        shooter.veloTimer.reset();
        while(opModeIsActive() && !isStopRequested()){

            drive.update();


            switch (pathState) {
                case RING1:
                    detector.phoneCam.stopStreaming();

                    //  drive to shoot first powershot
                    Trajectory shootFirstPowershot = drive.trajectoryBuilder(startPose)
                            .addDisplacementMarker(() -> {
                                //drop wobble
                                wobblemech.grip();
                            })
                            .splineToConstantHeading(new Vector2d(54,-19), Math.toRadians(0))
                            .addTemporalMarker(.5, () ->{
                                linkage.flickerOut();
                                shooter.motorSetPIDpower(.435);
                                linkage.raise();
                                angleFlap.setPosition(.43);
                                wobblemech.idle();

                            })
                            .build();

                    drive.followTrajectory(shootFirstPowershot);
                    sleep(200);
                    actuateFlicker();
                    sleep(600);

                    //strafe to middle powershot
                    Trajectory shootSecondPowershot = drive.trajectoryBuilder(shootFirstPowershot.end())
                            .lineToConstantHeading(new Vector2d(54,-10.5),new MinVelocityConstraint(Arrays.asList(
                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(45, DriveConstants.TRACK_WIDTH)
                            )
                            ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();
                    drive.followTrajectory(shootSecondPowershot);
                    sleep(200);
                    actuateFlicker();
                    sleep(600);

                    //Third Powershot
                    Trajectory shootThirdPowershot = drive.trajectoryBuilder(shootSecondPowershot.end())
                            .lineToConstantHeading(new Vector2d(54,-1),new MinVelocityConstraint(Arrays.asList(
                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(45, DriveConstants.TRACK_WIDTH)
                            )
                            ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();
                    drive.followTrajectory(shootThirdPowershot);
                    sleep(200);
                    actuateFlicker();
                    sleep(600);


                    //Drop Wobble
                    Trajectory dropWobble = drive.trajectoryBuilder(shootThirdPowershot.end())
                            .lineTo(new Vector2d(82,2))
                            .addDisplacementMarker(() -> {
                                //drop wobble
                                shooter.setNoPIDPower(0);
                                wobblemech.letGo();
                                linkage.lower();
                            })

                            .build();
                    drive.followTrajectory(dropWobble);

                    //Intake and pickup second wobble

                    Trajectory intakePickupWobbleTwo = drive.trajectoryBuilder(dropWobble.end())
                            .addDisplacementMarker(() -> {
                                //drop wobble
                                wobblemech.retract();
                            })
                            .lineToLinearHeading(new Pose2d(56,8, Math.toRadians(180)))
                            .addTemporalMarker(1.5, () ->{
                                wobblemech.extend();
                                wobblemech.letGo();

                            })
                            .build();

                    intake.setPower(.8);
                    drive.followTrajectory(intakePickupWobbleTwo);

                    //pickup wobble
                    Trajectory pickupWobble = drive.trajectoryBuilder(intakePickupWobbleTwo.end())
                            .lineToLinearHeading(new Pose2d(30,20, Math.toRadians(135)),new MinVelocityConstraint(Arrays.asList(
                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                            )
                            ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addDisplacementMarker(() -> {
                                wobblemech.grip();
                            })

                            .build();
                    drive.followTrajectory(pickupWobble);

                    Trajectory driveToShoot = drive.trajectoryBuilder(pickupWobble.end())
                            .addDisplacementMarker(() -> {
                                wobblemech.idle();
                                linkage.raise();
                                shooter.motorSetPIDpower(.9);
                                angleFlap.setPosition(.38);

                            })
                            .lineToLinearHeading(new Pose2d(56,12,Math.toRadians(350)),new MinVelocityConstraint(Arrays.asList(
                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                            )
                            ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addDisplacementMarker(() -> {
                                wobblemech.grip();
                            })
                            .build();


                    intake.stop();
                    drive.followTrajectory(driveToShoot);
                    actuateFlicker();
                    sleep(600);
                    sleep(200);



                    Trajectory dropWobble2 = drive.trajectoryBuilder(driveToShoot.end())
                            .lineTo(new Vector2d(83,6))
                            .addDisplacementMarker(() -> {
                                wobblemech.letGo();
                                shooter.motorSetPIDpower(0);
                            })
                            .addDisplacementMarker(() -> {
                                wobblemech.retract();
                            })
                            .build();

                    drive.followTrajectory(dropWobble2);
                    sleep(300);


                    Trajectory park = drive.trajectoryBuilder(dropWobble2.end())
                            .lineTo(new Vector2d(72,6))
                            .build();

                    drive.followTrajectory(park);











                    stop();
                    break;
                case RING4:
                   double x = 1;


                   stop();
                    break;
                case RING0:


                   double y = 2;
                    break;
                case CONTINUE:
                    double s = 5;
                    break;
            }
        }
    }
    public void actuateFlicker() {
        linkage.flickerIn();
        sleep(150);
        linkage.flickerOut();
    }
}
