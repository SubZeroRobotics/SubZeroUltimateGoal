package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "testDetection")
public class testdetection extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double ringCount = 0;

         RingDetectorV3 detector = new RingDetectorV3("BLUE",hardwareMap, telemetry);


        FtcDashboard.getInstance().startCameraStream(detector.phoneCam, 25);

        ringCount =  detector.getRingPosition();

        telemetry.addData("RingCount", ringCount);
        runOpMode();
        while(opModeIsActive()){

        }

    }
}
