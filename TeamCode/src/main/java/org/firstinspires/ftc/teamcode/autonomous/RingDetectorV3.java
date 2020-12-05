package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingDetectorV3 {
    OpenCvCamera phoneCam;
    HardwareMap hwmp;
    Telemetry telemetry;
    Double ringCount = 0.0;


    public RingDetectorV3(String Color, HardwareMap hw, Telemetry telemetry){
        this.hwmp = hw;
        telemetry = telemetry;
    }


    public double getRingPosition(){
        int cameraMonitorViewId = hwmp.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmp.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.setPipeline(new RingDetectorV3.RingDetectingPipeline());
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        return ringCount;
    }


    class RingDetectingPipeline extends OpenCvPipeline {


        Mat YCbCr = new Mat();
        Mat upperCrop = new Mat();
        Mat lowerCrop = new Mat();
        Mat outputMat = new Mat();


        @Override
        public Mat processFrame(Mat input) {
            //convert mat.
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            //copy input mat onto output
            input.copyTo(outputMat);


            Rect rect = new Rect((int) Math.round(YCbCr.rows() * .18), (int) Math.round(YCbCr.cols() * .25), 119, 69);

            Rect rect2 = new Rect((int) Math.round(YCbCr.rows() * .18), (int) Math.round(YCbCr.cols() * .32), 119, 20);


            Imgproc.rectangle(outputMat, rect, new Scalar(0, 0, 255), 2);
            Imgproc.rectangle(outputMat, rect2, new Scalar(0, 0, 255), 2);


            //crop the top part of the stack, and the bottom part of the stack.
            lowerCrop = YCbCr.submat(rect2);
            upperCrop = YCbCr.submat(rect);


            //extract the Cb channel
            Core.extractChannel(lowerCrop, lowerCrop, 2);
            Core.extractChannel(upperCrop, upperCrop, 2);

            //take average
            Scalar lowerColor = Core.mean(lowerCrop);
            Scalar upperColor = Core.mean(upperCrop);

            double lowColor = lowerColor.val[0];
            double upColor = upperColor.val[0];

            //check which one to determine the

            if (lowColor > 15 && upColor > 15 && lowColor < 130 && upColor < 130) {
                ringCount = 4.0;
            } else if (lowColor > 10 && upColor < 15 && lowColor > 10 && upColor < 15) {
                ringCount = 0.0;
            } else {
                ringCount = 1.0;
            }

            Imgproc.putText(outputMat,
                    "Rings : " + ringCount.toString(),
                    new Point(rect.x + 10, rect.y - 10),
                    5,
                    .75,
                    new Scalar(255, 0, 255),
                    1);

            return outputMat;
        }
    }
}

