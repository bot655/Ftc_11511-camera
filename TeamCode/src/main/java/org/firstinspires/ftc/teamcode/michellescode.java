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
import org.opencv.core.Rect;
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


@TeleOp(name = "MICHELLLES CODE", group = "Testing")
public class michellescode extends LinearOpMode {
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
                    telemetry.addLine("640 x 480 failed, trying 1280 x 720 ...");
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
        }

        camera.stopStreaming();
    }

    static class ColorClassificationPipeline extends OpenCvPipeline {
        private Mat lab = new Mat();
        private List<Mat> labChannels = new ArrayList<>();
        private Mat hsv = new Mat();
        private Mat maskRed = new Mat();
        private Mat hierarchy = new Mat();
        private final List<MatOfPoint> contours = new ArrayList<>();
        private final Mat maskYellow = new Mat();
        private final Mat maskRed1 = new Mat();
        private final Mat maskRed2 = new Mat();
        private final Mat maskBlue = new Mat();
        private Mat labelMap = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);
            Core.split(lab, labChannels);
            Core.normalize(labChannels.get(0), labChannels.get(0), 0, 255, Core.NORM_MINMAX);
            Core.merge(labChannels, lab);
            Imgproc.cvtColor(lab, input, Imgproc.COLOR_Lab2RGB);

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Thresholds for yellow
            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

            // Thresholds for red
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), maskRed1);
            Core.inRange(hsv, new Scalar(160, 100, 100), new Scalar(179, 255, 255), maskRed2);
            Core.bitwise_or(maskRed1, maskRed2, maskRed);

            // Thresholds for blue
            Scalar lowerBlue = new Scalar(100, 100, 100);
            Scalar upperBlue = new Scalar(130, 255, 255);
            Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);

            if (labelMap.empty() || !labelMap.size().equals(input.size())) {
                labelMap = Mat.zeros(input.size(), CvType.CV_8UC3);
            } else {
                labelMap.setTo(new Scalar(0, 0, 0));
            }

            input.copyTo(labelMap);

            // Red contours with rotated bounding box
            contours.clear();
            Imgproc.findContours(maskRed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > 500) {
                    RotatedRect rotRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                    Point[] vertices = new Point[4];
                    rotRect.points(vertices);
                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(labelMap, vertices[i], vertices[(i + 1) % 4], new Scalar(255, 0, 0), 2);
                    }
                    Size size = rotRect.size;
                    double w = size.width;
                    double h = size.height;
                    Point center = rotRect.center;

                    String dims = String.format(Locale.US, "%.0f x %.0f px", w, h);
                    int font = Imgproc.FONT_HERSHEY_SIMPLEX;
                    double scale = 0.6;
                    int thickness = 2;
                    Scalar textColor = new Scalar(255, 255, 255);
                    Scalar bgColor = new Scalar(0, 0, 0);
                    int[] baseLine = new int[1];
                    Size textSize = Imgproc.getTextSize(dims, font, scale, thickness, baseLine);
                    baseLine[0] += thickness;

                    Point textOrg = new Point(
                            center.x - textSize.width / 2,
                            center.y + textSize.height / 2
                    );
                    Imgproc.rectangle(
                            labelMap,
                            new Point(textOrg.x, textOrg.y + baseLine[0]),
                            new Point(textOrg.x + textSize.width, textOrg.y - textSize.height),
                            bgColor,
                            Imgproc.FILLED
                    );
                    Imgproc.putText(labelMap, dims, textOrg, font, scale, textColor, thickness);
                }
            }

            // Blue contours with rotated bounding box
            contours.clear();
            Imgproc.findContours(maskBlue, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > 500) {
                    RotatedRect rotRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                    Point[] vertices = new Point[4];
                    rotRect.points(vertices);
                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(labelMap, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 0, 255), 2);
                    }
                    Size size = rotRect.size;
                    double w = size.width;
                    double h = size.height;
                    Point center = rotRect.center;

                    String dims = String.format(Locale.US, "%.0f x %.0f px", w, h);
                    int font = Imgproc.FONT_HERSHEY_SIMPLEX;
                    double scale = 0.6;
                    int thickness = 2;
                    Scalar textColor = new Scalar(255, 255, 255);
                    Scalar bgColor = new Scalar(0, 0, 0);
                    int[] baseLine = new int[1];
                    Size textSize = Imgproc.getTextSize(dims, font, scale, thickness, baseLine);
                    baseLine[0] += thickness;

                    Point textOrg = new Point(
                            center.x - textSize.width / 2,
                            center.y + textSize.height / 2
                    );
                    Imgproc.rectangle(
                            labelMap,
                            new Point(textOrg.x, textOrg.y + baseLine[0]),
                            new Point(textOrg.x + textSize.width, textOrg.y - textSize.height),
                            bgColor,
                            Imgproc.FILLED
                    );
                    Imgproc.putText(labelMap, dims, textOrg, font, scale, textColor, thickness);
                }
            }

            return labelMap;
        }
    }
}
