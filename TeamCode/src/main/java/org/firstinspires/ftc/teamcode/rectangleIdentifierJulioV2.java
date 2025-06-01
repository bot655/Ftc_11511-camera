package org.firstinspires.ftc.teamcode;

import android.util.Size;

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

@TeleOp(name = "fucked camera detection", group = "Testing")
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
        private final int WIDTH_PX = (int) (TARGET_WIDTH_MM * PIXELS_PER_MM);
        private final int HEIGHT_PX = (int) (TARGET_HEIGHT_MM * PIXELS_PER_MM);
        private final int TOLERANCE_PX = 20;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), maskRed1);
            Core.inRange(hsv, new Scalar(160, 100, 100), new Scalar(179, 255, 255), maskRed2);
            Core.bitwise_or(maskRed1, maskRed2, maskRed);

            Scalar lowerBlue = new Scalar(100, 100, 100);
            Scalar upperBlue = new Scalar(130, 255, 255);
            Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);

            if (labelMap.empty() || !labelMap.size().equals(input.size())) {
                labelMap = Mat.zeros(input.size(), CvType.CV_8UC3);
            } else {
                labelMap.setTo(new Scalar(0, 0, 0));
            }
            input.copyTo(labelMap);

            // Helper function
            processContours(maskRed, labelMap, new Scalar(255, 0, 0), "Red");
            processContours(maskBlue, labelMap, new Scalar(0, 0, 255), "Blue");

            return labelMap;
        }

        private void processContours(Mat mask, Mat output, Scalar color, String label) {
            contours.clear();

            // Morphology to split grouped blobs
            Mat morphed = new Mat();
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 7;
            Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);  // erosion + dilation

            Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                double area = Imgproc.contourArea(contour);
                if (area > 200) {
                    double widthMM = rect.width / PIXELS_PER_MM;
                    double heightMM = rect.height / PIXELS_PER_MM;

                    Imgproc.rectangle(output, rect, color, 2);
                    String dims = String.format(Locale.ENGLISH, "%.1fmm x %.1fmm", widthMM, heightMM);
                    Imgproc.putText(output, dims, new Point(rect.x, rect.y - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);

                    if (Math.abs(rect.width - WIDTH_PX) <= TOLERANCE_PX &&
                            Math.abs(rect.height - HEIGHT_PX) <= TOLERANCE_PX) {
                        Imgproc.rectangle(output, rect, new Scalar(0, 255, 0), 3);
                        Imgproc.putText(output, "Target Size", new Point(rect.x, rect.y + rect.height + 15),
                                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
                    }

                    System.out.printf("%s RECT: %d x %d px (%.1fmm x %.1fmm), Area: %.1f\n",
                            label, rect.width, rect.height, widthMM, heightMM, area);
                }
            }

            morphed.release();
            kernel.release();
        }

    }
}

