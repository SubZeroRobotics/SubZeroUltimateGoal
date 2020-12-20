package org.firstinspires.ftc.teamcode.autonomous.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;

public class AutoTrajectories {

    //dt;
     public SampleMecanumDrive drive;

    //trajectory array list
    public ArrayList<Trajectory> trajectoryRing1 = new ArrayList<Trajectory>();
    public ArrayList<Trajectory> trajectoryRing0 = new ArrayList<Trajectory>();
    public ArrayList<Trajectory> trajectoryRing4 = new ArrayList<Trajectory>();
    public ArrayList<Trajectory> trajectoryContinue = new ArrayList<Trajectory>();


    //robot start position. setting it at 0,0,0 for now;
    Pose2d startPose;
    Shooter shooter;
    public Linkage linkage;
    public Servo angleFlap;
    public HardwareMap hardwareMap;


    public double linkageWait = .75;



    public AutoTrajectories(HardwareMap hw, SampleMecanumDrive drive, Shooter shooter, Linkage linkage){
        this.drive = drive;
        this.shooter = shooter;
        this.linkage = linkage;
        this.hardwareMap = hw;
        angleFlap = hardwareMap.get(Servo.class, "flap");
    }

    public void initTrajectories(Pose2d startPose){
        this.startPose = startPose;
        //case 4
        Trajectory trajFour1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(54,-6),0)
                .splineTo(new Vector2d(115,18), Math.toRadians(45))
                .addTemporalMarker(.5, () ->{
                    shooter.setNoPIDPower(.875);
                })
                .build();
        trajectoryRing4.add(trajFour1);


        Trajectory trajFour2 = drive.trajectoryBuilder(trajFour1.end(), true)
                .splineTo(new Vector2d(51,-10), Math.toRadians(181))
                .addTemporalMarker(.5, () -> {
                    // raise the linkage
                    linkage.raise();
                    angleFlap.setPosition(.345);
                })
                .build();
        trajectoryRing4.add(trajFour2);

        Trajectory trajFour3 = drive.trajectoryBuilder(trajFour2.end())
                .lineToLinearHeading(new Pose2d(110.5,28, 180))
                .build();
     //   trajectoryRing4.add(trajFour3);


        Trajectory trajFour4 = drive.trajectoryBuilder(trajFour3.end())
                .lineToLinearHeading(new Pose2d(75, 180))
                .build();
        trajectoryRing4.add(trajFour4);


    }







}
