package org.firstinspires.ftc.teamcode.testing;

import android.os.health.PackageHealthStats;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "GTPTest", group = "testing")
public class GoToPointTesting extends LinearOpMode {
    SampleMecanumDrive drive;



    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        PathFollower follower = new PathFollower(hardwareMap,drive);
        follower.reset();
        waitForStart();
        while(opModeIsActive()) {
            follower.goToPoint(new Pose2d(48,-12,0),1);


        }
    }
}
