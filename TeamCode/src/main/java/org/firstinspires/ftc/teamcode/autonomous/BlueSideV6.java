package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;


@Autonomous(name = "BlueSideV6")
public class BlueSideV6 extends LinearOpMode {
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
        linkage = new Linkage(hardwareMap,shooter, elapsedTime, elapsedTime2, .7,.38,.28,.52 );
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobblemech = new Wobblemech(hardwareMap, elapsedTime3);
        angleFlap = hardwareMap.get(Servo.class, "flap");




        RingDetectorV3 detector = new RingDetectorV3("BLUE",hardwareMap, telemetry,rows,rectCols,rect2Cols);
        detector.init();
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        BlueSideV5.State pathState = BlueSideV5.State.IDLE;


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

//        if(ringCount == 4.0){
//            pathState = BlueSideV5.State.RING4;
//        }else if(ringCount == 1.0){
//            pathState = BlueSideV5.State.RING1;
//        }else{
//            pathState = BlueSideV5.State.RING0;
//        }

        pathState = BlueSideV5.State.RING1;

        waitForStart();

        shooter.veloTimer.reset();
        drive.update();

        while(opModeIsActive()) {
            switch (pathState) {
                case RING1:
                    shooter.internalPID(1150);
                    //  drive to shoot first powershot
                    Trajectory shootFirstPowershot = drive.trajectoryBuilder(startPose)
                            .addDisplacementMarker(() -> {
                                //drop wobble
                                wobblemech.grip();
                            })
                            .splineToConstantHeading(new Vector2d(54,-19), Math.toRadians(0))
                            .addTemporalMarker(.5, () ->{
                                linkage.flickerOut();
                                linkage.raise();
                                angleFlap.setPosition(.4);
                                wobblemech.idle();

                            })
                            .build();
                    drive.followTrajectory(shootFirstPowershot);
                    sleep(200);
                    actuateFlicker();
                    sleep(600);
                    stop();
                    break;
                case RING0:
                    double vaa = 1;
                    break;
                case RING4:
                    double vvv = 1;
                    break;
                case IDLE:
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
