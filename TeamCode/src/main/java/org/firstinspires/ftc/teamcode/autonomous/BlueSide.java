package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class BlueSide extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake  noodleIntake = new Intake(hardwareMap);
        VisionByAverage ringDetector = new VisionByAverage("BLUE");

        Pose2d startPose = new Pose2d(0,0,0);

        waitForStart();

        if(isStopRequested()) return;

        //drive to the first depositing location


        Trajectory depositFirstWobblyGoal = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0,0), Math.toRadians(0))
                .splineTo(new Vector2d(0,0), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    //deposit wobbly goal
                    depositWobblyGoal();
                })
                .build();
        drive.followTrajectory(depositFirstWobblyGoal);

        Trajectory pickupSecondWobble = drive.trajectoryBuilder(depositFirstWobblyGoal.end(), true)
                .splineTo(new Vector2d(0,0), Math.toRadians(0))
                .splineTo(new Vector2d(0,0), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    //grab wobbly goal
                })
                .build();
        drive.followTrajectory(pickupSecondWobble);
        //turn on intkake, get ready to intake rings
        noodleIntake.setPower(1);

        Trajectory intakeAutoRings = drive.trajectoryBuilder(pickupSecondWobble.end(), false)
                .splineTo(new Vector2d(0,0), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                  //stop intake
                    noodleIntake.stop();
                })
                .addDisplacementMarker(() -> {
                    shooter.actuateShootingSequence(1);
                })
                .build();

        drive.followTrajectory(intakeAutoRings);

        Trajectory park = drive.trajectoryBuilder(intakeAutoRings.end())
                .forward(0)
                .build();
        drive.followTrajectory(park);


















    }

    public static void depositWobblyGoal(){

    }



}
