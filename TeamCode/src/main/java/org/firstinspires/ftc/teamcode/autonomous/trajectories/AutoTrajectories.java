package org.firstinspires.ftc.teamcode.autonomous.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;

import java.util.ArrayList;

public class AutoTrajectories {

    //dt;
     public SampleMecanumDrive drive;

    //trajectory array list
    public ArrayList<Trajectory> trajectoryRing1 = new ArrayList<Trajectory>();
    public ArrayList<Trajectory> trajectoryRing0 = new ArrayList<Trajectory>();
    public ArrayList<Trajectory> trajectoryRing4 = new ArrayList<Trajectory>();


    //robot start position. setting it at 0,0,0 for now;
    Pose2d startPose;
    Shooter shooter;
    public Linkage linkage;
    public Servo angleFlap;
    public Wobblemech wobblemech;
    public HardwareMap hardwareMap;


    public double linkageWait = .75;



    public AutoTrajectories(HardwareMap hw, SampleMecanumDrive drive, Shooter shooter, Linkage linkage, Wobblemech wobblemech){
        this.drive = drive;
        this.shooter = shooter;
        this.linkage = linkage;
        this.wobblemech = wobblemech;
        this.hardwareMap = hw;
        angleFlap = hardwareMap.get(Servo.class, "flap");
    }

    public void initTrajectories(Pose2d startPose){
        this.startPose = startPose;
        /*
          case 4
         */



        Trajectory trajFour1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(54.5,-12),0)
                .addTemporalMarker(.5, () ->{
                    shooter.setNoPIDPower(.9);
                    linkage.raise();
                    angleFlap.setPosition(.435);
                })
                .addTemporalMarker(.1, () ->{
                    wobblemech.idle();
                })
                .build();
        trajectoryRing4.add(trajFour1);


        Trajectory trajFour2 = drive.trajectoryBuilder(trajFour1.end(), false)
                .splineTo(new Vector2d(115,18), Math.toRadians(45))
                .addTemporalMarker(.5, () -> {
                    // raise the linkage
                    linkage.lower();
                    angleFlap.setPosition(.425);
                })
                .addDisplacementMarker(() -> {
                    //drop wobble
                    wobblemech.letGo();
                    angleFlap.setPosition(.35);

                })
                .build();
        trajectoryRing4.add(trajFour2);

        Trajectory trajFour3 = drive.trajectoryBuilder(trajFour2.end())
                .lineToLinearHeading(new Pose2d(40,-10, Math.toRadians(90)))
                .addTemporalMarker(.5, () -> {
                    // bring back the wobble
                    wobblemech.letGo();
                    wobblemech.retract();

                })
                .build();
          trajectoryRing4.add(trajFour3);


        Trajectory trajFour4 = drive.trajectoryBuilder(trajFour3.end())
                .lineToConstantHeading(new Vector2d(40,3), new DriveConstraints(17, 17.50393309, 0.0,
                        Math.toRadians(359.7672057337132), Math.toRadians(359.7672057337132), 0.0))
                .build();
        trajectoryRing4.add(trajFour4);

        Trajectory trajFour5 = drive.trajectoryBuilder(trajFour4.end())
                .lineToConstantHeading(new Vector2d(40,14), new DriveConstraints(5, 5.50393309, 0.0,
                        Math.toRadians(359.7672057337132), Math.toRadians(359.7672057337132), 0.0))
                .addTemporalMarker(.5, () -> {
                    wobblemech.extend();
                    wobblemech.letGo();
                })
                .build();
        trajectoryRing4.add(trajFour5);

        Trajectory trajFour6 = drive.trajectoryBuilder(trajFour5.end())
                .lineToConstantHeading(new Vector2d(16,2), new DriveConstraints(55, 55.50393309, 0.0,
                        Math.toRadians(359.7672057337132), Math.toRadians(359.7672057337132), 0.0))

                    .addTemporalMarker(.2, () -> {
                    shooter.setNoPIDPower(.9);
                 })
                .addDisplacementMarker(() -> {
                    //drop wobble
                    wobblemech.grip();
                })
                .build();
        trajectoryRing4.add(trajFour6);


        //shooting rings in high goal
        Trajectory trajFour7 = drive.trajectoryBuilder(trajFour6.end())
                .lineToLinearHeading(new Pose2d(54.5,8, Math.toRadians(0)))
                .addTemporalMarker(.2, () -> {
                    // raise the linkage
                  //  shooter.setNoPIDPower(.9);
                    angleFlap.setPosition(.425);
                    linkage.raise();
                })
                .build();
        trajectoryRing4.add(trajFour7);


        // drop wobble
        Trajectory trajFour8 = drive.trajectoryBuilder(trajFour7.end())
                .splineTo(new Vector2d(115,18), Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    wobblemech.letGo();
                })
                .build();
        trajectoryRing4.add(trajFour8);

        //park
        Trajectory trajFour9 = drive.trajectoryBuilder(trajFour8.end())
                .lineTo(new Vector2d(68,-10))
                .addTemporalMarker(.5, () -> {
                    // raise the linkage
                    //  shooter.setNoPIDPower(.9);
                    linkage.lower();
                    wobblemech.retract();
                })
                .build();
        trajectoryRing4.add(trajFour9);



           /*
          case 1
         */

        //power shots
        Trajectory trajOne1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(54.5,-12),0)
                .addTemporalMarker(.5, () ->{
                    shooter.setNoPIDPower(.9);
                    linkage.raise();
                    angleFlap.setPosition(.435);
                })
                .addTemporalMarker(.1, () ->{
                    wobblemech.idle();
                })
                .build();
        trajectoryRing1.add(trajOne1);

        //drop wobble
        Trajectory trajOne2 = drive.trajectoryBuilder(trajOne1.end(), false)
                .splineTo(new Vector2d(88,-6), Math.toRadians(45))
                .addTemporalMarker(.5, () -> {
                    // raise the linkage
                    linkage.lower();
                    angleFlap.setPosition(.425);
                })
                .addDisplacementMarker(() -> {
                    //drop wobble
                    wobblemech.letGo();
                    angleFlap.setPosition(.35);

                })
                .build();
        trajectoryRing1.add(trajOne2);

        //come to stack
        Trajectory trajOne3 = drive.trajectoryBuilder(trajOne2.end())
                .lineToLinearHeading(new Pose2d(40,-10, Math.toRadians(90)))
                .addTemporalMarker(.5, () -> {
                    // bring back the wobble
                    wobblemech.letGo();
                    wobblemech.retract();

                })
                .build();
        trajectoryRing1.add(trajOne3);


        Trajectory trajOne4 = drive.trajectoryBuilder(trajOne3.end())
                .lineToConstantHeading(new Vector2d(40,3), new DriveConstraints(65, 65.50393309, 0.0,
                        Math.toRadians(359.7672057337132), Math.toRadians(359.7672057337132), 0.0))
                .addTemporalMarker(.5, () -> {
                    wobblemech.extend();
                    wobblemech.letGo();
                })
                .build();
        trajectoryRing1.add(trajOne4);

//        Trajectory trajOne5 = drive.trajectoryBuilder(trajOne4.end())
//                .lineToConstantHeading(new Vector2d(40,14), new DriveConstraints(5, 5.50393309, 0.0,
//                        Math.toRadians(359.7672057337132), Math.toRadians(359.7672057337132), 0.0))
//                .addTemporalMarker(.5, () -> {
//                    wobblemech.extend();
//                    wobblemech.letGo();
//                })
//                .build();
//        trajectoryRing4.add(trajOne5);

        Trajectory trajOne5 = drive.trajectoryBuilder(trajOne4.end())
                .lineToConstantHeading(new Vector2d(16,2), new DriveConstraints(55, 55.50393309, 0.0,
                        Math.toRadians(359.7672057337132), Math.toRadians(359.7672057337132), 0.0))

                .addTemporalMarker(.2, () -> {
                    shooter.setNoPIDPower(.9);
                })
                .addDisplacementMarker(() -> {
                    //drop wobble
                    wobblemech.grip();
                })
                .build();
        trajectoryRing1.add(trajOne5);


        //shooting rings in high goal
        Trajectory trajOne6 = drive.trajectoryBuilder(trajOne5.end())
                .lineToLinearHeading(new Pose2d(54.5,8, Math.toRadians(0)))
                .addTemporalMarker(.2, () -> {
                    // raise the linkage
                    //  shooter.setNoPIDPower(.9);
                    angleFlap.setPosition(.425);
                    linkage.raise();
                })
                .build();
        trajectoryRing1.add(trajOne6);


        // drop wobble
        Trajectory trajOne7 = drive.trajectoryBuilder(trajOne6.end())
                .splineTo(new Vector2d(89,-6), Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    wobblemech.letGo();
                })
                .build();
        trajectoryRing1.add(trajOne7);

        //park
        Trajectory trajOne8 = drive.trajectoryBuilder(trajOne7.end())
                .lineTo(new Vector2d(68,-10))
                .addTemporalMarker(.5, () -> {
                    // raise the linkage
                    //  shooter.setNoPIDPower(.9);
                    linkage.lower();
                    wobblemech.retract();
                })
                .build();
        trajectoryRing1.add(trajOne8);


         /*
          case 0
         */


        Trajectory trajZero1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(54.5,-12),0)
                .addTemporalMarker(.5, () ->{
                    shooter.setNoPIDPower(.9);
                    linkage.raise();
                    angleFlap.setPosition(.435);
                })
                .addTemporalMarker(.1, () ->{
                    wobblemech.idle();
                })
                .build();
        trajectoryRing0.add(trajZero1);

        //drop wobble
        Trajectory trajZero2 = drive.trajectoryBuilder(trajZero1.end(), false)
                .splineTo(new Vector2d(70,18), Math.toRadians(45))
                .addTemporalMarker(.5, () -> {
                    // raise the linkage
                    linkage.lower();
                    angleFlap.setPosition(.425);
                })
                .addDisplacementMarker(() -> {
                    //drop wobble
                    wobblemech.letGo();
                    angleFlap.setPosition(.35);

                })
                .build();
        trajectoryRing0.add(trajZero2);

//        //come to stack
//        Trajectory trajZero3 = drive.trajectoryBuilder(trajZero2.end())
//                .lineToLinearHeading(new Pose2d(40,-10, Math.toRadians(90)))
//                .addTemporalMarker(.5, () -> {
//                    // bring back the wobble
//                    wobblemech.letGo();
//                    wobblemech.retract();
//
//                })
//                .build();
//        trajectoryRing0.add(trajZero3);


//        Trajectory trajZero4 = drive.trajectoryBuilder(trajZero2.end())
//                .lineToConstantHeading(new Vector2d(40,3), new DriveConstraints(65, 65.50393309, 0.0,
//                        Math.toRadians(359.7672057337132), Math.toRadians(359.7672057337132), 0.0))
//                .addTemporalMarker(.5, () -> {
//                    wobblemech.extend();
//                    wobblemech.letGo();
//                })
//                .build();
//        trajectoryRing0.add(trajZero4);


        Trajectory trajZero4 = drive.trajectoryBuilder(trajZero2.end())
                .lineToLinearHeading(new Pose2d(16,2, Math.toRadians(90)), new DriveConstraints(55, 55.50393309, 0.0,
                        Math.toRadians(359.7672057337132), Math.toRadians(359.7672057337132), 0.0))

                .addTemporalMarker(.2, () -> {
                    //shooter.setNoPIDPower(.9);
                    wobblemech.extend();
                })

                .build();
        trajectoryRing0.add(trajZero4);


        //shooting rings in high goal



        // drop wobble
        Trajectory trajZero5 = drive.trajectoryBuilder(trajZero4.end())
                .splineTo(new Vector2d(66,19), Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    wobblemech.letGo();
                })
                .build();
        trajectoryRing0.add(trajZero5);

        //park
        Trajectory trajZero6 = drive.trajectoryBuilder(trajZero5.end())
                .lineTo(new Vector2d(68,-10))
                .addTemporalMarker(.5, () -> {
                    // raise the linkage
                    //  shooter.setNoPIDPower(.9);
                    linkage.lower();
                    wobblemech.retract();
                })
                .build();
        trajectoryRing0.add(trajZero6);


































    }







}
