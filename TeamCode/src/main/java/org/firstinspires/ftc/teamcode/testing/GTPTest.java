package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "GTPtest", group = "testing")
public class GTPTest extends LinearOpMode {
    SampleMecanumDrive drive;

    Vector2d targetVector = new Vector2d(124,-25);


    public boolean autoTurn = false;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        GTPV3 follower = new GTPV3(hardwareMap,drive);
        follower.reset();
        waitForStart();


        while(opModeIsActive()) {
            telemetry.addData("Angle To Turn", Math.toDegrees(getAngleToTurn()));
            telemetry.addData("pose", drive.getPoseEstimate());

            telemetry.update();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y ,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );



            if(gamepad1.a){
                autoTurn = true;
            }

            while(autoTurn){
                follower.turnToAbsolute(Math.toDegrees(getAngleToTurn()), 1);
                if(!gamepad1.atRest()){
                    autoTurn = false;
                    break;
                }
            }


            drive.update();

        }


    }


    public double getAngleToTurn(){
        return Math.atan2( targetVector.getY() - drive.getPoseEstimate().getY(),targetVector.getX() - drive.getPoseEstimate().getX());
    }


}
