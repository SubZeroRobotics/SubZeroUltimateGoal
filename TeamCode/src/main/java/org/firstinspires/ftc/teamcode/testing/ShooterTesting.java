package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Linkage;
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

    public ElapsedTime timer = new ElapsedTime();
    public static double anglerposition = 0;
    public static double targetPower = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    TelemetryPacket packet = new TelemetryPacket();




    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        SimpleServo angler = new SimpleServo(hardwareMap, "flap");
        SimpleServo indexer = new SimpleServo(hardwareMap, "pusher");
        Linkage linkage = new Linkage(hardwareMap, push,pull);
        angler.setPosition(1);
        waitForStart();
        timer.startTime();
        while (opModeIsActive()) {
            if (gamepad1.left_trigger > .8) {
                //shooter.start();
                shooter.setNoPIDPower(1);
            } else {
               // shooter.stop();
                shooter.setNoPIDPower(0);
            }
            shooter.update();
        }


        if(gamepad1.a){

        }


       // packet.put("Desired Power", 6000);

      //  packet.put("Motor1 Power", shooter.currentRPM);

      //  dashboard.sendTelemetryPacket(packet);


    }
}
