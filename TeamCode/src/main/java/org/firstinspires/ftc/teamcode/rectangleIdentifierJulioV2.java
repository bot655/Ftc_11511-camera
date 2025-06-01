package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Locale;

@TeleOp(name = "Identify Rectangles", group = "Testing")
public class rectangleIdentifierJulioV2 extends LinearOpMode {
    String allianceColour = "Blue";
    OpenCvCamera camera;
    boolean cameraOpened = false;

    ColorClassificationPipeline pipeline;

    @Override
    public void runOpMode() {
        int viewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, viewId);

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
                    telemetry.addLine("640 x 480 failed");
                    telemetry.update();
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
        }

        camera.stopStreaming();
    }

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

        // Calibration assumption: 2 pixels per mm (adjust for your setup)
        private final double PIXELS_PER_MM = 2.0;
        private final int TARGET_WIDTH_MM = 88;
        private final int TARGET_HEIGHT_MM = 38;
        private final int WIDTH_PX = (int)(TARGET_WIDTH_MM * PIXELS_PER_MM);
        private final int HEIGHT_PX = (int)(TARGET_HEIGHT_MM * PIXELS_PER_MM);
        private final int TOLERANCE_PX = 20;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);
            maskYellow.setTo(new Scalar(0));
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

            maskRed1.setTo(new Scalar(0));
            maskRed2.setTo(new Scalar(0));
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), maskRed1);
            Core.inRange(hsv, new Scalar(160, 100, 100), new Scalar(179, 255, 255), maskRed2);
            Core.bitwise_or(maskRed1, maskRed2, maskRed);

            Scalar lowerBlue = new Scalar(100, 100, 100);
            Scalar upperBlue = new Scalar(130, 255, 255);
            maskBlue.setTo(new Scalar(0));
            Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);

            if (labelMap.empty() || !labelMap.size().equals(input.size())) {
                labelMap = Mat.zeros(input.size(), CvType.CV_8UC3);
            } else {
                labelMap.setTo(new Scalar(0, 0, 0));
            }
            input.copyTo(labelMap);

            // RED
            contours.clear();
            Imgproc.findContours(maskRed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                if (Imgproc.contourArea(contour) > 500 &&
                        Math.abs(rect.width - WIDTH_PX) <= TOLERANCE_PX &&
                        Math.abs(rect.height - HEIGHT_PX) <= TOLERANCE_PX) {
                    Imgproc.rectangle(labelMap, rect, new Scalar(255, 0, 0), 2);
                    String dims = String.format(Locale.ENGLISH, "%.1fmm x %.1fmm", rect.width / PIXELS_PER_MM, rect.height / PIXELS_PER_MM);
                    Imgproc.putText(labelMap, dims, new Point(rect.x, rect.y - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
                }
            }

            // BLUE
            contours.clear();
            Imgproc.findContours(maskBlue, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                if (Imgproc.contourArea(contour) > 500 &&
                        Math.abs(rect.width - WIDTH_PX) <= TOLERANCE_PX &&
                        Math.abs(rect.height - HEIGHT_PX) <= TOLERANCE_PX) {
                    Imgproc.rectangle(labelMap, rect, new Scalar(0, 0, 255), 2);
                    String dims = String.format(Locale.ENGLISH, "%.1fmm x %.1fmm", rect.width / PIXELS_PER_MM, rect.height / PIXELS_PER_MM);
                    Imgproc.putText(labelMap, dims, new Point(rect.x, rect.y - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
                }
            }

            return labelMap;
        }
    }
}
