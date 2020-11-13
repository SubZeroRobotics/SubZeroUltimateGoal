package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


@Config

public class RingDetectorV2 {
    OpenCvCamera phoneCam;
    HardwareMap hwmp;
    Telemetry telemetry;
    Double ringCount = 0.0;


    public RingDetectorV2(String Color, HardwareMap hw, Telemetry telemetry){
        this.hwmp = hw;
        telemetry = telemetry;
    }


    public double getRingPosition(){
        int cameraMonitorViewId = hwmp.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmp.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.setPipeline(new RingDetectorV2.RingDetectingPipeline());
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();

        return ringCount;
    }

    Mat YCbCr = new Mat();
    Mat upperCrop = new Mat();
    Mat lowerCrop = new Mat();
    Mat outputMat = new Mat();

    public int[] lowerRect = {
            (int)Math.round(YCbCr.rows() * .25),
            (int)Math.round(YCbCr.rows() * .25),
            (int)Math.round(YCbCr.cols() * .25),
            (int)Math.round(YCbCr.cols() * .25)
    };

  public  int[] upperRect = {
          (int)Math.round(YCbCr.rows() * .25),
          (int)Math.round(YCbCr.rows() * .25),
          (int)Math.round(YCbCr.cols() * .25),
          (int)Math.round(YCbCr.cols() * .25)
    };




    class RingDetectingPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            //convert mat.
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            //copy input mat onto output
            input.copyTo(outputMat);

            Imgproc.rectangle(outputMat,
                    new Point(lowerRect[0], lowerRect[2]),
                    new Point(lowerRect[1], lowerRect[3]),
                    new Scalar(0,0,255),
                    2);

            Imgproc.rectangle(outputMat,
                    new Point(upperRect[0], upperRect[2]),
                    new Point(upperRect[1], upperRect[3]),
                    new Scalar(0,0,255),
                    2);




            //crop the top part of the stack, and the bottom part of the stack.
            lowerCrop = YCbCr.submat(lowerRect[0], lowerRect[1], lowerRect[2], lowerRect[3]);
            upperCrop = YCbCr.submat(upperRect[0], upperRect[1], upperRect[2], upperRect[3]);



            //extract the Cb channel
            Core.extractChannel(lowerCrop, lowerCrop, 2);
            Core.extractChannel(upperCrop,upperCrop,2);

            //take average
            Scalar lowerColor = Core.mean(lowerCrop);
            Scalar upperColor = Core.mean(upperCrop);

            double lowColor = lowerColor.val[0];
            double upColor = upperColor.val[0];

            //check which one to determine the

            if(lowColor > 15 && upColor > 15 && lowColor < 100 && upColor < 100){
                ringCount = 4.0;
            }else if(lowColor > 10 && upColor < 15 && lowColor > 10 && upColor < 15){
                ringCount = 0.0;
            }else{
                ringCount = 1.0;
            }

            Imgproc.putText(outputMat,
                    "Rings : " + ringCount.toString(),
                    new Point(0,0),
                    5,
                    1,
                    new Scalar(0,0,255),
                    3);

            return outputMat;
        }
    }
}
