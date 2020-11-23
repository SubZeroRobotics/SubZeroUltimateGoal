package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp(name = "ShooterTesting", group = "Test")
public class ShooterTesting extends LinearOpMode {

    public Shooter shooter;

    public static double kp;
    public static double ki, kd, kf;

    public static double push;
    public static double pull;

    private static double COUNTS_PER_REV = 28;
    private static double ROTATIONS_PER_MINUTE = 5480;

    public static double anglerposition = 0;
    public static double targetPower = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        SimpleServo angler = new SimpleServo(hardwareMap, "flap");
        SimpleServo indexer = new SimpleServo(hardwareMap, "pusher");
        angler.setPosition(1);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                shooter.start();
            } else {
                shooter.stop();
            }
            shooter.update();
        }


    }
}
