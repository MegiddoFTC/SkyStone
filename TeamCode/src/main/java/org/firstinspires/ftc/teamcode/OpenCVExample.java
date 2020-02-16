package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
@TeleOp(name = "OpenCV")
public class OpenCVExample extends LinearOpMode{
    OpenCvCamera phoneCam;
    double gg;
    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SamplePipeline());
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            if(gamepad1.a) {
                phoneCam.stopStreaming();
                //webcam.closeCameraDevice();
            }
            else if(gamepad1.x) {
                phoneCam.pauseViewport();
            }
            else if(gamepad1.y) {
                phoneCam.resumeViewport();
            }
            else if (gamepad1.b){
                phoneCam.closeCameraDevice();
            }
            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(
                    input,
                    new Point(163, 108),
                    new Point(167, 132),
                    new Scalar(255,140,0), 2);

            Imgproc.rectangle(
                    input,
                    new Point(93, 108),
                    new Point(97, 132),
                    new Scalar(255,0,0), 2);

            double[] x = input.get(110, 165);
            double[] x2 = input.get(115,165);
            double[] x3 = input.get(120,165);
            double[] x4 = input.get(125,165);
            double[] x5 = input.get(130,165);

            double[] y = input.get(110,95);
            double[] y2 = input.get(115,95);
            double[] y3 = input.get(120,95);
            double[] y4 = input.get(125,95);
            double[] y5 = input.get(130,95);

            if ((y[0]+y[1]+y[2])<90||(y2[0]+y2[1]+y2[2])<90||(y3[0]+y3[1]+y3[2])<90||(y4[0]+y4[1]+y4[2])<90||(y5[0]+y5[1]+y5[2])<90){
                gg=69;
            }else if((x[0]+x[1]+x[2])<90||(x2[0]+x2[1]+x2[2])<90||(x3[0]+x3[1]+x3[2])<90||(x4[0]+x4[1]+x4[2])<90||(x5[0]+x5[1]+x5[2])<90){
                gg=420;
            }else {
                gg=254;
            }
            telemetry.addData("ClrY", y[0] + " " + y[1] + " " + y[2]);
            telemetry.addData("ClrX", x[0] + " " + x[1] + " " + x[2]);
            telemetry.addData("gg",gg);
            telemetry.update();
            return input;
        }
    }
}
