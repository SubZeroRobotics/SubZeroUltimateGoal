package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class TeleOp extends LinearOpMode {

    GamepadEx g1;
    GamepadEx g2;
    Shooter shooter;
    Intake noodleIntake;
    double frontLeftPower, frontRightPower, backLeftPower,backRightPower = 0;
    SampleMecanumDrive drive;
    ButtonReader buttonA;
    ButtonReader buttonB;
    Pose2d robotPose = new Pose2d();
    public static  Pose2d shooterPose = new Pose2d(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {

         g1 = new GamepadEx(gamepad1);
         g2 = new GamepadEx(gamepad2);
         shooter = new Shooter(hardwareMap);
         noodleIntake = new Intake(hardwareMap);

         buttonA = new ButtonReader(g1, GamepadKeys.Button.A);
         buttonB = new ButtonReader(g1, GamepadKeys.Button.B);


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            waitForStart();

            //--------------------robot movement------------------------
                drive.setWeightedDrivePower(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(
                                -g1.getLeftY(),
                                -g1.getLeftX(),
                                -g1.getRightX()
                        )
                );
                drive.update();
                robotPose =  drive.getPoseEstimate();
        }
        //---------------------------subsystem----------------------------

        //TURN TO GOAL
        if(buttonA.wasJustPressed()){
            drive.turn(turnToGoal(robotPose));
        }

        //SHOOTING SEQUENCE
        if(buttonB.wasJustPressed()){
            shooter.actuateShootingSequence(1);
        }

        //INTAKE
        noodleIntake.setPower(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        noodleIntake.setPower(-g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));


    }

    public static double turnToGoal(Pose2d robotPose){
        //convert coordinates

        //calc desired heading
        double x = robotPose.getX();
        double y = robotPose.getY();
        double turnAngle = Math.atan2(shooterPose.getY() - robotPose.getY(), shooterPose.getX() - robotPose.getX());


        return  turnAngle;
    }


}
