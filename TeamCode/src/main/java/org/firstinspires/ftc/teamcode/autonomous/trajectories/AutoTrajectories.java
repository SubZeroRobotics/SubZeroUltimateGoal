package org.firstinspires.ftc.teamcode.autonomous.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class AutoTrajectories {

    //dt;
     public static SampleMecanumDrive drive;

    //trajectory array list
    public ArrayList<Trajectory> trajectoryRing1 = new ArrayList<Trajectory>();
    public ArrayList<Trajectory> trajectoryRing0 = new ArrayList<Trajectory>();
    public ArrayList<Trajectory> trajectoryRing4 = new ArrayList<Trajectory>();
    public ArrayList<Trajectory> trajectoryContinue = new ArrayList<Trajectory>();


    //robot start position. setting it at 0,0,0 for now;
    Pose2d startPose;


    public AutoTrajectories(SampleMecanumDrive drive){
        this.drive = drive;
    }

    public void initTrajectories(Pose2d startPose){
        this.startPose = startPose;
        //case 4
        Trajectory trajFour1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(54,-6),0)
                .splineTo(new Vector2d(110.5,24), 45)
                .build();
        Trajectory trajFour2 = drive.trajectoryBuilder(trajFour1.end(), false)
                .lineToLinearHeading(new Pose2d(30,24, 135))
                .build();
        Trajectory trajFour3 = drive.trajectoryBuilder(trajFour2.end())
                .lineToLinearHeading(new Pose2d(110.5,28, 180))
                .build();
        Trajectory trajFour4 = drive.trajectoryBuilder(trajFour3.end())
                .lineToLinearHeading(new Pose2d(75,28, 180))
                .build();

        trajectoryRing1.add(trajFour1);
        trajectoryRing1.add(trajFour2);
        trajectoryRing1.add(trajFour3);
        trajectoryRing1.add(trajFour4);

    }







}
