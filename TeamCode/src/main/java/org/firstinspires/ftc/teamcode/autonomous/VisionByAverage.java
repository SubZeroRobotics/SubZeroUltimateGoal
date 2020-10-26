package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.ExpansionHubEx;


/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 */
@TeleOp
public class VisionByAverage extends LinearOpMode {
    OpenCvCamera phoneCam;
    public static int one = 25;
    public static int two = 25;
    public static int three = 100;
    public static int four = 100;
    double range1;
    double range2;
    double range3;
    double range4;
    double range5;
    double range6;
    double rings = 0;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.setPipeline(new SamplePipeline());
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
//            telemetry.addData("Frame Count", phoneCam.getFrameCount());
//            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
//            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
//            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
//            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
//            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
//            telemetry.update();

            if (gamepad1.a) {
                phoneCam.stopStreaming();
                //phoneCam.closeCameraDevice();
            }
            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused = false;
        Mat yCbCrChan2Mat = new Mat();
        private Mat cropRegionMat = new Mat();
        private Mat ExtractMat = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb); //converts rgb to hsv
            int[] rectangle = {
                    one,
                    two,
                    three,
                    four
            };
            Imgproc.rectangle(
                    yCbCrChan2Mat,
                    new Point(
                            rectangle[0],
                            rectangle[1]),

                    new Point(
                            rectangle[2],
                            rectangle[3]),
                    new Scalar(255, 0, 0), 4);
            // cropRegionMat = yCbCrChan2Mat.submat(rectangle[1], rectangle[3], rectangle[0], rectangle[2]);
            cropRegionMat = yCbCrChan2Mat.submat(rectangle[0], rectangle[3], rectangle[1], rectangle[3]);

            Core.extractChannel(cropRegionMat, ExtractMat, 2);

            Scalar mean = Core.mean(ExtractMat);
            double averageColor = mean.val[0];

            if (averageColor <= 115.0 && averageColor >= 90.0) {
                rings = 1;
            } else if (averageColor <= 90.0) {
                rings = 4;
            } else {
                rings = 0;
            }

            telemetry.addData("Average Color", averageColor);
            telemetry.addData("Ring", rings);
            telemetry.update();
            return ExtractMat;
        }

        @Override
        public void onViewportTapped() {


            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                phoneCam.pauseViewport();
            } else {
                phoneCam.resumeViewport();
            }
        }
    }
}
