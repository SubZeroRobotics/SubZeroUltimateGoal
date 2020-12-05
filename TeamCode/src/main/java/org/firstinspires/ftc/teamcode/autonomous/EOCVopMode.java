package org.firstinspires.ftc.teamcode.autonomous;

/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

import static java.lang.Thread.sleep;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 */
@Autonomous(name = "RingDetecting")
public class EOCVopMode extends OpMode
{
    OpenCvCamera phoneCam = null;
    double ringCount = 0;


    @Override
    public void init() {

    }

    @Override
    public void init_loop(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.setPipeline(new RingDetectingPipeline());

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });


    }

    @Override
    public void loop() {

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

            telemetry.addData("Rings:", ringCount);

            return outputMat;
        }
    }
}