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
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "camera detection??", group = "Testing")
public class FuckThisShitBro extends LinearOpMode {
    String allianceColour = "Blue";
    OpenCvWebcam camera;
    boolean cameraOpened = false;

    ColorClassificationPipeline pipeline;

    @Override
    public void runOpMode() {
        int viewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, viewId);
        // Note: setLensIntrinsics is not supported on external webcams in EasyOpenCV
        // Intrinsics calibration is only available for internal phone camera

        pipeline = new ColorClassificationPipeline();
        camera.setPipeline(pipeline);
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                cameraOpened = true;
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
        private Mat maskRed1 = new Mat();
        private Mat maskRed2 = new Mat();
        private Mat maskBlue = new Mat();
        private Mat maskYellow = new Mat();
        private Mat hierarchy = new Mat();
        private List<MatOfPoint> contours = new ArrayList<>();

        // Calibration: 2 pixels per mm
        private final double PIXELS_PER_MM = 2.0;
        private final int TARGET_WIDTH_MM = 88;
        private final int TARGET_HEIGHT_MM = 38;
        private final int WIDTH_PX = (int) (TARGET_WIDTH_MM * PIXELS_PER_MM);
        private final int HEIGHT_PX = (int) (TARGET_HEIGHT_MM * PIXELS_PER_MM);
        private final int TOLERANCE_PX = 20;

        private Mat labelMap = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // threshold colors
            Core.inRange(hsv, new Scalar(20, 100, 100), new Scalar(30, 255, 255), maskYellow);
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), maskRed1);
            Core.inRange(hsv, new Scalar(160, 100, 100), new Scalar(179, 255, 255), maskRed2);
            Core.bitwise_or(maskRed1, maskRed2, maskRed);
            Core.inRange(hsv, new Scalar(100, 100, 100), new Scalar(130, 255, 255), maskBlue);

            // setup output
            if (labelMap.empty() || !labelMap.size().equals(input.size())) {
                labelMap = Mat.zeros(input.size(), CvType.CV_8UC3);
            } else {
                labelMap.setTo(new Scalar(0, 0, 0));
            }
            input.copyTo(labelMap);

            // draw rotated boxes
            processContours(maskRed, labelMap, new Scalar(255, 0, 0), "Red");
            processContours(maskBlue, labelMap, new Scalar(0, 0, 255), "Blue");

            return labelMap;
        }

        private void processContours(Mat mask, Mat output, Scalar drawColor, String label) {
            contours.clear();

            // clean up
            Mat morphed = new Mat();
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 7));
            Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);

            Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 200) {
                    RotatedRect rRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                    Point[] vertices = new Point[4];
                    rRect.points(vertices);

                    // draw rotated rect
                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(output, vertices[i], vertices[(i + 1) % 4], drawColor, 2);
                    }

                    // measure
                    double widthMM = rRect.size.width / PIXELS_PER_MM;
                    double heightMM = rRect.size.height / PIXELS_PER_MM;
                    String dims = String.format(Locale.ENGLISH, "%.1fmm x %.1fmm", widthMM, heightMM);
                    Imgproc.putText(output, dims, rRect.center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,
                            new Scalar(255, 255, 255), 1);

                    // highlight target
                    if (Math.abs(rRect.size.width - WIDTH_PX) <= TOLERANCE_PX &&
                            Math.abs(rRect.size.height - HEIGHT_PX) <= TOLERANCE_PX) {
                        for (int i = 0; i < 4; i++) {
                            Imgproc.line(output, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 3);
                        }
                        Imgproc.putText(output, "Target Size",
                                new Point(rRect.center.x - 30, rRect.center.y + rRect.size.height/2 + 15),
                                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
                    }

                    System.out.printf("%s RECT: %.0f x %.0f px (%.1fmm x %.1fmm), Area: %.1f\n",
                            label, rRect.size.width, rRect.size.height, widthMM, heightMM, area);
                }
            }

            morphed.release();
            kernel.release();
        }
    }
}
