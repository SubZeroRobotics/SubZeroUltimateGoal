package org.firstinspires.ftc.teamcode.autonomous.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Wobblemech;

import java.util.ArrayList;
import java.util.Arrays;

public class AutoTrajectoriesHigh {

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

    double highGoalFlap = .345;
    double powershotFlap = .38;


    public double linkageWait = .75;



    public AutoTrajectoriesHigh(HardwareMap hw, SampleMecanumDrive drive, Shooter shooter, Linkage linkage, Wobblemech wobblemech){
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


        //drive to shoot 3 rings
        //0
        Trajectory trajFour1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(26,12),0)
                .addTemporalMarker(.5, () ->{
                    shooter.setNoPIDPower(.9);
                    linkage.raise();
                    angleFlap.setPosition(.3425);
                })
                .addTemporalMarker(.1, () ->{
                    wobblemech.idle();
                })
                .build();
        trajectoryRing4.add(trajFour1);

        //knock rings over
        //1
        Trajectory trajFour15 = drive.trajectoryBuilder(trajFour1.end())
                .lineToConstantHeading(new Vector2d(38,12),  new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2, () ->{
                    angleFlap.setPosition(highGoalFlap);
                })

                .build();

        trajectoryRing4.add(trajFour15);
        //intake rings
        //2
        Trajectory trajFour16 = drive.trajectoryBuilder(trajFour15.end())

                .lineToConstantHeading(new Vector2d(46,12), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
                )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(.5, () ->{
                   shooter.setNoPIDPower(.9);
                })


                .build();
        trajectoryRing4.add(trajFour16);


        //drop wobble off
        //3
        Trajectory trajFour2 = drive.trajectoryBuilder(trajFour16.end(), false)
                .lineTo(new Vector2d(110,30))
                .addTemporalMarker(.1, () -> {
                    // raise the linkage
                    angleFlap.setPosition(.45);
                })
                .addDisplacementMarker(() -> {
                    //drop wobble
                    wobblemech.letGo();
                    angleFlap.setPosition(highGoalFlap);

                })
                .build();
        trajectoryRing4.add(trajFour2);

        //4
        //drop off

        Trajectory trajFour5 = drive.trajectoryBuilder(trajFour2.end())
                .lineToLinearHeading(new Pose2d(16,4, Math.toRadians(90)), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
                )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(.5, () -> {
                    wobblemech.extend();
                    wobblemech.letGo();
                })
                .build();
        trajectoryRing4.add(trajFour5);

        //drop off
        // 5
        Trajectory trajFour6 = drive.trajectoryBuilder(trajFour5.end())
                .splineTo(new Vector2d(38,12),Math.toRadians(0), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
                )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(.2, () -> {
                    shooter.setNoPIDPower(.9);
                })
                .addTemporalMarker(.3, () -> {
                    // raise the linkage
                    //  shooter.setNoPIDPower(.9);
                    angleFlap.setPosition(highGoalFlap);
                    linkage.raise();
                    shooter.setNoPIDPower(1);
                })
                .build();
        trajectoryRing4.add(trajFour6);


        //shooting rings in high goal
        Trajectory trajFour7 = drive.trajectoryBuilder(trajFour6.end())

                .lineToLinearHeading(new Pose2d(110,30, Math.toRadians(30)))
                .addDisplacementMarker( () -> {
                    wobblemech.extend();
                })
                .addDisplacementMarker(() -> {
                    //drop wobble
                    wobblemech.letGo();
                })

                .build();
        trajectoryRing4.add(trajFour7);


        // drop wobble


        //park
        Trajectory trajFour9 = drive.trajectoryBuilder(trajFour7.end())
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
                .splineToConstantHeading(new Vector2d(26,12),0)
                .addTemporalMarker(.2, () ->{
                    shooter.setNoPIDPower(.9);
                    linkage.raise();
                    angleFlap.setPosition(highGoalFlap);
                })
                .addTemporalMarker(.1, () ->{
                    wobblemech.idle();
                })
                .build();
        trajectoryRing1.add(trajOne1);

        Trajectory trajOne15 = drive.trajectoryBuilder(trajOne1.end())
                .lineToConstantHeading(new Vector2d(38,12))
                .addTemporalMarker(2, () ->{
                    linkage.raise();
                    angleFlap.setPosition(highGoalFlap);
                })
                .build();

        trajectoryRing1.add(trajOne15);



        //drop wobble
        Trajectory trajOne2 = drive.trajectoryBuilder(trajOne15.end(), false)
                .splineTo(new Vector2d(94,0), Math.toRadians(45))

                .addDisplacementMarker(() -> {
                    //drop wobble
                    wobblemech.letGo();
                    angleFlap.setPosition(.5);

                })
                .build();
        trajectoryRing1.add(trajOne2);


        Trajectory trajOne5 = drive.trajectoryBuilder(trajOne2.end())
                .lineToLinearHeading(new Pose2d(16,4,Math.toRadians(90)), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
                )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    //drop wobble
                    wobblemech.grip();
                })
                .addTemporalMarker(.5, () -> {
                    wobblemech.extend();
                    wobblemech.letGo();
                })
                .build();
        trajectoryRing1.add(trajOne5);

        // drop wobble
        Trajectory trajOne7 = drive.trajectoryBuilder(trajOne5.end())
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
                .splineToConstantHeading(new Vector2d(54.5,14),0)
                .addTemporalMarker(.5, () ->{
                    shooter.setNoPIDPower(1);
                    linkage.raise();
                    angleFlap.setPosition(.33);
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
                    angleFlap.setPosition(.5);
                })
                .addDisplacementMarker(() -> {
                    //drop wobble
                    wobblemech.letGo();
                    angleFlap.setPosition(highGoalFlap);

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
                .lineToLinearHeading(new Pose2d(16,4, Math.toRadians(90)), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
                )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addTemporalMarker(.5, () -> {
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
