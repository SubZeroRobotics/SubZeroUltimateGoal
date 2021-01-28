package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.trajectories.AutoTrajectories;
import org.firstinspires.ftc.teamcode.autonomous.trajectories.AutoTrajectoriesHigh;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;
import org.firstinspires.ftc.teamcode.util.TrajectoryStorage;


@Autonomous(name = "BlueSideV3")
public class BlueSideV3 extends LinearOpMode {

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
        linkage = new Linkage(hardwareMap,shooter, elapsedTime, elapsedTime2, .7,.38,.3,.52 );
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobblemech = new Wobblemech(hardwareMap, elapsedTime3);
        AutoTrajectoriesHigh autoTrajectories = new AutoTrajectoriesHigh(hardwareMap, drive, shooter, linkage, wobblemech);
        autoTrajectories.initTrajectories(startPose);



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
            detector.phoneCam.stopStreaming();

            drive.update();


            switch (pathState) {
                case RING1:

                    wobblemech.grip();
                    sleep(100);
                    drive.followTrajectory(autoTrajectories.trajectoryRing1.get(0));

                    //powershots
                    actuateFlicker();
                    sleep(600);
                    actuateFlicker();
                    sleep(600);
                    actuateFlicker();
                    sleep(600);
                    actuateFlicker();
                    sleep(600);

                    linkage.lower();
                    sleep(200);
                    intake.setPower(1);
                    drive.followTrajectory(autoTrajectories.trajectoryRing1.get(1));
                    sleep(800);
                    actuateFlicker();
                    sleep(600);
                    intake.stop();


                    telemetry.addData("Pose", drive.getPoseEstimate());
                    telemetry.update();
                    drive.followTrajectory(autoTrajectories.trajectoryRing1.get(2));
                    //drive to rings
                    drive.followTrajectory(autoTrajectories.trajectoryRing1.get(3));

                    sleep(250);
                    wobblemech.idle();
                    sleep(100);

                    drive.followTrajectory(autoTrajectories.trajectoryRing1.get(4));

                    drive.followTrajectory(autoTrajectories.trajectoryRing1.get(5));

                    stop();
                    break;
                case RING4:
                    //drive to powershot
                    wobblemech.grip();
                    sleep(100);
                    drive.followTrajectory(autoTrajectories.trajectoryRing4.get(0));
                    //powershots
                    //powershots
                    actuateFlicker();
                    sleep(300);
                    actuateFlicker();
                    sleep(300);
                    actuateFlicker();
                    sleep(300);
                    actuateFlicker();
                    sleep(300);
                    linkage.lower();
                    intake.setPower(0);
                    //go to drop wobbly
                    drive.followTrajectory(autoTrajectories.trajectoryRing4.get(1));
                    intake.setPower(1);
                    drive.followTrajectory(autoTrajectories.trajectoryRing4.get(2));
                    sleep(500);
                    intake.setPower(-1);
                    sleep(300);
                    linkage.raise();
                    sleep(1000);
                    actuateFlicker();
                    sleep(300);
                    actuateFlicker();
                    sleep(300);
                    actuateFlicker();
                    sleep(300);
                    actuateFlicker();
                    sleep(300);
                    linkage.lower();
                    sleep(100);
                    intake.setPower(1);
                    drive.followTrajectory(autoTrajectories.trajectoryRing4.get(3));
                    intake.setPower(0);
                    sleep(300);
                    drive.followTrajectory(autoTrajectories.trajectoryRing4.get(4));
                    wobblemech.grip();
                    sleep(200);
                    wobblemech.idle();
                    sleep(200);
                    drive.followTrajectory(autoTrajectories.trajectoryRing4.get(5));
                    sleep(1000);
                    actuateFlicker();
                    sleep(150);
                    actuateFlicker();
                    sleep(150);
                    actuateFlicker();
                    sleep(150);
                    actuateFlicker();
                    sleep(150);
                    drive.followTrajectory(autoTrajectories.trajectoryRing4.get(6));
                    drive.followTrajectory(autoTrajectories.trajectoryRing4.get(7));


                    stop();
                    break;


                case RING0:
                    wobblemech.grip();
                    sleep(100);
                    drive.followTrajectory(autoTrajectories.trajectoryRing0.get(0));
                    //powershots
                    actuateFlicker();
                    sleep(600);
                    actuateFlicker();
                    sleep(600);
                    actuateFlicker();
                    sleep(600);
                    actuateFlicker();
                    sleep(600);
                    telemetry.addData("Pose", drive.getPoseEstimate());
                    telemetry.update();




                    //go to drop wobbly
                    drive.followTrajectory(autoTrajectories.trajectoryRing0.get(1));
                    shooter.motorSetPIDpower(0);
                    //drive to rings
                    drive.followTrajectory(autoTrajectories.trajectoryRing0.get(2));
                    sleep(200);
                    wobblemech.grip();
                    sleep(250);
                    wobblemech.idle();
                    sleep(100);
                    drive.followTrajectory(autoTrajectories.trajectoryRing0.get(3));
                    drive.followTrajectory(autoTrajectories.trajectoryRing0.get(4));
                    shooter.motorSetPIDpower(0);
                    stop();


                    break;
                case CONTINUE:
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
