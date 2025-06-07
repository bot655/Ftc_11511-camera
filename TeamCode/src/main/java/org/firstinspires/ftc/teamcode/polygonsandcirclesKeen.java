package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "polygonsandcirclesKeen", group = "Testing")
public class polygonsandcirclesKeen extends LinearOpMode {
    OpenCvCamera camera;
    boolean cameraOpened = false;

    @Override
    public void runOpMode() {
        int viewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, viewId);

        ShapeDetectionPipeline pipeline = new ShapeDetectionPipeline();
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
                    try {
                        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                        cameraOpened = true;
                    } catch (Exception fallbackError) {
                        telemetry.addLine("Camera failed to start");
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
            }
            telemetry.update();
            sleep(50);
        }

        camera.stopStreaming();
    }

    static class ShapeDetectionPipeline extends OpenCvPipeline {
        private final Mat hsv = new Mat();
        private final Mat maskRed = new Mat();
        private final Mat maskBlue = new Mat();
        private final Mat output = new Mat();
        private final Mat morphed = new Mat();
        private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 7));

        private static final double CIRCULARITY_THRESHOLD = 0.75;
        private static final double RECTANGLE_ASPECT_TOLERANCE = 0.3;
        private static final int MIN_AREA = 500;
        private static final double PIXELS_PER_MM = 2.0;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsv, new Scalar(0, 50, 50), new Scalar(15, 255, 255), maskRed);
            Core.inRange(hsv, new Scalar(90, 50, 50), new Scalar(140, 255, 255), maskBlue);

            input.copyTo(output);

            processMask(maskRed, output, new Scalar(255, 0, 0));
            processMask(maskBlue, output, new Scalar(0, 0, 255));

            maskRed.release();
            maskBlue.release();
            hsv.release();

            return output;
        }

        private void processMask(Mat mask, Mat outputImage, Scalar color) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);
            Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < MIN_AREA) {
                    contour.release();
                    continue;
                }

                MatOfPoint2f curve = new MatOfPoint2f(contour.toArray());
                RotatedRect rRect = Imgproc.minAreaRect(curve);

                MatOfPoint2f approxCurve = new MatOfPoint2f();
                double epsilon = 0.02 * Imgproc.arcLength(curve, true);
                Imgproc.approxPolyDP(curve, approxCurve, epsilon, true);

                double perimeter = Imgproc.arcLength(curve, true);
                double circularity = (4 * Math.PI * area) / (perimeter * perimeter);

                String shapeLabel;
                Point[] points = approxCurve.toArray();

                if (circularity > CIRCULARITY_THRESHOLD) {
                    shapeLabel = "Circle";
                } else if (points.length == 4) {
                    double aspectRatio = rRect.size.width / (double)rRect.size.height;
                    double ratio = Math.max(aspectRatio, 1.0 / aspectRatio);
                    if (ratio < 1.0 + RECTANGLE_ASPECT_TOLERANCE) {
                        shapeLabel = "Square";
                    } else {
                        shapeLabel = "Rectangle";
                    }
                } else if (points.length == 3) {
                    shapeLabel = "Triangle";
                } else if (points.length > 4) {
                    shapeLabel = "Polygon";
                } else {
                    shapeLabel = "Unknown";
                }

                Point center = rRect.center;
                double widthMM = rRect.size.width / PIXELS_PER_MM;
                double heightMM = rRect.size.height / PIXELS_PER_MM;
                double areaMM = area / (PIXELS_PER_MM * PIXELS_PER_MM);

                Point[] vertices = new Point[4];
                rRect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(outputImage, vertices[i], vertices[(i+1)%4], color, 2);
                }

                String shapeText = String.format(Locale.ENGLISH, "%s (%.0f mm²)", shapeLabel, areaMM);
                String dimText = String.format(Locale.ENGLISH, "%.1f x %.1f mm", widthMM, heightMM);

                Imgproc.putText(outputImage, shapeText, center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
                Imgproc.putText(outputImage, dimText, new Point(center.x, center.y + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(200, 200, 255), 1);

                curve.release();
                approxCurve.release();
                contour.release();
            }

            hierarchy.release();
        }
    }
}