package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

// import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.UvcCameraName;
@TeleOp(name = "Identify Rectangles", group = "Testing")
public class rectangleIdentifierJulio extends LinearOpMode {
    String allianceColour = "Blue";
    OpenCvCamera camera;
    boolean cameraOpened = false;

    Rect centerbox = new Rect(280, 200, 80, 80); // detection zone

    ColorClassificationPipeline pipeline;

    @Override
    public void runOpMode() {
        int viewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, viewId);

        // Initialize pipeline
        pipeline = new ColorClassificationPipeline();
        camera.setPipeline(pipeline);
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);


        telemetry.addLine("Waiting for start...");
        telemetry.update();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                try {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    cameraOpened = true;
                } catch (Exception e) {
                    telemetry.addLine("1920 x 1080 failed, trying 1280 x 720 (640 x 480) ...");
                    telemetry.update();
                    try {
                        camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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

            telemetry.addData("Alliance", allianceColour);
            telemetry.update();

            // sleep(10);
        }

        camera.stopStreaming();
    }

    // Color classification pipeline: scans the whole frame and classifies pixels by color, draws bounding box around red objects
    static class ColorClassificationPipeline extends OpenCvPipeline {
        private Mat hsv = new Mat();
        private Mat maskRed = new Mat();
        private Mat hierarchy = new Mat();
        private java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();

        private Mat maskYellow = new Mat();
        private Mat maskRed1 = new Mat();
        private Mat maskRed2 = new Mat();
        private Mat maskBlue = new Mat();
        private Mat labelMap = new Mat();

        // Define the desired size of the bounding box in mm
        private static final double DESIRED_WIDTH_MM = 88.0;
        private static final double DESIRED_HEIGHT_MM = 38.0;

        // Assume you have calculated the pixel-per-mm ratio for the camera
        private static final double PIXELS_PER_MM_X = 10.0;  // Example: 10 pixels per mm along X-axis
        private static final double PIXELS_PER_MM_Y = 10.0;  // Example: 10 pixels per mm along Y-axis

        private static final int BOUNDING_BOX_WIDTH_PIXELS = (int) (DESIRED_WIDTH_MM * PIXELS_PER_MM_X);
        private static final int BOUNDING_BOX_HEIGHT_PIXELS = (int) (DESIRED_HEIGHT_MM * PIXELS_PER_MM_Y);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Thresholds for yellow
            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);
            maskYellow.setTo(new Scalar(0));
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

            // Thresholds for red (two ranges because red wraps around hue)
            maskRed1.setTo(new Scalar(0));
            maskRed2.setTo(new Scalar(0));
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), maskRed1);
            Core.inRange(hsv, new Scalar(160, 100, 100), new Scalar(179, 255, 255), maskRed2);
            Core.bitwise_or(maskRed1, maskRed2, maskRed);

            // Thresholds for blue
            Scalar lowerBlue = new Scalar(100, 100, 100);
            Scalar upperBlue = new Scalar(130, 255, 255);
            maskBlue.setTo(new Scalar(0));
            Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);

            // Combine masks into label map (for debugging or classification purposes)
            if (labelMap.empty() || !labelMap.size().equals(input.size())) {
                labelMap = Mat.zeros(input.size(), CvType.CV_8UC3);
            } else {
                labelMap.setTo(new Scalar(0, 0, 0));
            }
            input.copyTo(labelMap);

            // Red: label = 2, draw bounding boxes
            contours.clear();
            Imgproc.findContours(maskRed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : contours) {
                // Instead of drawing the bounding box around the contour's actual size, use the fixed size
                Rect fixedRect = new Rect(
                        100,  // X position of the bounding box (can be adjusted)
                        100,  // Y position of the bounding box (can be adjusted)
                        BOUNDING_BOX_WIDTH_PIXELS,
                        BOUNDING_BOX_HEIGHT_PIXELS
                );
                Imgproc.rectangle(labelMap, fixedRect, new Scalar(255, 0, 0), 2); // Red boundary
            }

            // Blue: draw bounding boxes
            contours.clear();
            Imgproc.findContours(maskBlue, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : contours) {
                // Similarly, use a fixed-size bounding box for blue objects
                Rect fixedRect = new Rect(
                        200,  // X position of the bounding box (can be adjusted)
                        200,  // Y position of the bounding box (can be adjusted)
                        BOUNDING_BOX_WIDTH_PIXELS,
                        BOUNDING_BOX_HEIGHT_PIXELS
                );
                Imgproc.rectangle(labelMap, fixedRect, new Scalar(0, 0, 255), 2); // Blue boundary
            }

            return labelMap;
        }
    }
}