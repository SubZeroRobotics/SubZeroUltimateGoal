package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.trajectories.AutoTrajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TrajectoryStorage;

@Autonomous(name = "BlueSideV2")
public class BlueSideV2 extends LinearOpMode {

    public enum State{
        RING1,
        RING4,
        RING0,
        CONTINUE,
        IDLE;
    }


  //  State ring_position = State.RING4;

    public Linkage linkage;
    public Shooter shooter;

    public Pose2d startPose = new Pose2d(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        linkage = new Linkage(hardwareMap, 0.27,0.87, 0.12,0.32);
        shooter = new Shooter(hardwareMap);
        AutoTrajectories autoTrajectories = new AutoTrajectories(hardwareMap,drive,shooter,linkage);
        autoTrajectories.initTrajectories(startPose);


        //vision code here
        State pathState = State.RING4;
        waitForStart();


        if (isStopRequested()) return;

        switch(pathState){
            case RING1:
                //do something

            break;
            case RING4:
                //drive
                autoTrajectories.linkage.lower();
                sleep(500);
                drive.followTrajectory(autoTrajectories.trajectoryRing4.get(0));
                drive.followTrajectory(autoTrajectories.trajectoryRing4.get(1));
                linkage.flickerIn();
                sleep(100);
                sleep(200);
                linkage.flickerOut();
                sleep(5000);



                TrajectoryStorage storage = new TrajectoryStorage(drive.getPoseEstimate());

                pathState = State.CONTINUE;
                break;
            case RING0:

            break;
            case CONTINUE:
            break;
        }
    }
}
