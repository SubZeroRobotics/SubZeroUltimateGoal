package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
@Autonomous(name = "testDetection")
public class testdetection extends LinearOpMode {
    public static double rows = .12;
    public static double rectCols = .39;
    public static double rect2Cols = .46;
    double ringCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {

         RingDetectorV3 detector = new RingDetectorV3("BLUE",hardwareMap, telemetry,rows,rectCols,rect2Cols);
         detector.init();
         TelemetryPacket packet = new TelemetryPacket();
         FtcDashboard dashboard = FtcDashboard.getInstance();

        while(!isStarted()) {
            ringCount = detector.getRingPosition();
            FtcDashboard.getInstance().startCameraStream(detector.phoneCam, 20);


            packet.put("LowColor", detector.lowColor);
            packet.put("UpColor", detector.upColor);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("RingCount", ringCount);
            telemetry.addData("LowColor", detector.getVal());
            telemetry.addData("UpColor", detector.getVal2());
            telemetry.update();
        }


        waitForStart();
        while(opModeIsActive()){

        }

    }
}
