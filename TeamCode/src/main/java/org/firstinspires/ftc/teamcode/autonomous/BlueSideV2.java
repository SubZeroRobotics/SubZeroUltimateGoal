package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.trajectories.AutoTrajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "BlueSideV2")
public class BlueSideV2 extends LinearOpMode {

    public enum PathState{
        RING1,
        RING4,
        RING0,
        CONTINUE,
        IDLE;
    }



    public Pose2d startPose = new Pose2d(0,0,0);


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        AutoTrajectories autoTrajectories = new AutoTrajectories(drive);

        autoTrajectories.initTrajectories(startPose);
        

        //vision code here
        PathState pathState = PathState.RING4;
        waitForStart();


        if (isStopRequested()) return;

        switch(pathState){
            case RING1:
                drive.followTrajectory(autoTrajectories.trajectoryRing1.get(0));
                sleep(200);
                drive.followTrajectory(autoTrajectories.trajectoryRing1.get(1));
                sleep(100);
                drive.followTrajectory(autoTrajectories.trajectoryRing1.get(2));
                drive.followTrajectory(autoTrajectories.trajectoryRing1.get(3));
            break;

            case RING4:

            break;

            case RING0:

            break;

            case CONTINUE:

            break;
        }
    }
}
