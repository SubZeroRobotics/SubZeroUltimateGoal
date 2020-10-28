package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class BlueSide extends LinearOpMode {
       //creating ring state machine
       public static enum RingPosition {
            ONE,
            FOUR,
            ZERO,
            NULL,
            CONTINUE,
        };

       double amtOfRings;
       public RingPosition ringPosition;
       public RingDetector ringDetector;
       private double rings;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake  noodleIntake = new Intake(hardwareMap);
        ringDetector = new RingDetector("BLUE", hardwareMap, telemetry);

        RingPosition ringPosition = RingPosition.NULL;

        //determine the start position
        Pose2d startPose = new Pose2d(0,0,0);

        //do vision stuffs here
        rings = ringDetector.getRingPosition();

        if(rings == 1){
            ringPosition = RingPosition.ONE;

        }else if(rings == 4) {
            ringPosition = RingPosition.FOUR;
        }else{
            ringPosition = RingPosition.ZERO;
        }
        
        waitForStart();

        if(isStopRequested()) return;

        switch(ringPosition){
            //TODO: ADD POWER SHOT METHOD
            case FOUR:
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
                ringPosition = RingPosition.CONTINUE;

                break;
            case CONTINUE:
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

                break;

            case ONE:
                //TODO: ADD POWER SHOT METHOD
                Trajectory depositFirstWobblyGoalONE = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(1,1), Math.toRadians(0))
                        .splineTo(new Vector2d(0,0), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            //deposit wobbly goal
                            depositWobblyGoal();
                        })
                        .build();
                drive.followTrajectory(depositFirstWobblyGoalONE);

                Trajectory pickupSecondWobbleONE = drive.trajectoryBuilder(depositFirstWobblyGoalONE.end(), true)
                        .splineTo(new Vector2d(0,0), Math.toRadians(0))
                        .splineTo(new Vector2d(0,0), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            //grab wobbly goal
                        })
                        .build();
                drive.followTrajectory(pickupSecondWobbleONE);
                ringPosition = RingPosition.CONTINUE;

                break;

            case ZERO:
                //TODO: ADD POWER SHOT METHOD
                Trajectory depositFirstWobblyGoalZERO = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(1,0), Math.toRadians(0))
                        .splineTo(new Vector2d(0,0), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            //deposit wobbly goal
                            depositWobblyGoal();
                        })
                        .build();
                drive.followTrajectory(depositFirstWobblyGoalZERO);

                Trajectory pickupSecondWobbleZERO = drive.trajectoryBuilder(depositFirstWobblyGoalZERO.end(), true)
                        .splineTo(new Vector2d(0,0), Math.toRadians(0))
                        .splineTo(new Vector2d(0,0), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            //grab wobbly goal
                        })
                        .build();
                drive.followTrajectory(pickupSecondWobbleZERO);
                ringPosition = RingPosition.CONTINUE;
                break;
        }
    }
    public static void depositWobblyGoal(){
    }
}