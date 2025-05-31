package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

@TeleOp(name = "Box Average Color", group = "Testing")
public class AverageColorBigBlackBoxAKAnigger extends LinearOpMode {
    String allianceColour = "Blue";
    OpenCvCamera camera;
    boolean cameraOpened = false;

    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;
    Rect centerbox = new Rect(280, 200, 80, 80); // detection zone

    AverageColorPipeline pipeline;

    @Override
    public void runOpMode() {
        int viewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, viewId);

        // Initialize pipeline
        pipeline = new AverageColorPipeline(centerbox);
        camera.setPipeline(pipeline);

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                try {
                    // camera.setPixelFormat(OpenCvCamera.PixelFormat.MJPEG); // from yuy2 to mjpeg
                    // camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW); // more framerate?
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    cameraOpened = true;
                } catch (Exception e) {
                    telemetry.addLine("640 x 480 failed, trying 1920 x 1080 ...");
                    telemetry.update();
                    try {
                        // camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW); // more framerate?
                        camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                        cameraOpened = true;
                    } catch (Exception fallbackError) {
                        telemetry.addLine("Fallback resolution also failed.");
                        telemetry.update();
                    }
                }
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            if (!cameraOpened) {
                telemetry.addLine("Camera not opened");
                telemetry.update();
                continue;
            }

            if (gamepad1.x) allianceColour = "Blue";
            if (gamepad1.b) allianceColour = "Red";

            Scalar color = pipeline.getAvgColor();
            telemetry.addData("Box value: R", color.val[0]);
            telemetry.addData("Box value: G", color.val[1]);
            telemetry.addData("Box value: B", color.val[2]);

            String detectedColor = color.val[2] < color.val[0] ? "Red" : "Blue";
            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Alliance", allianceColour);
            telemetry.update();

            // sleep(10);
        }

        camera.stopStreaming();
    }

    // Inner pipeline class
    static class AverageColorPipeline extends OpenCvPipeline {
        private Scalar avgColor = new Scalar(0, 0, 0);
        private final Rect region;

        public AverageColorPipeline(Rect region) {
            this.region = region;
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat regionMat = input.submat(region);
            avgColor = Core.mean(regionMat);
            Imgproc.rectangle(input, region, new Scalar(255, 0, 0), 2);
            regionMat.release();
            return input;
        }

        public Scalar getAvgColor() {
            return avgColor;
        }
    }
}