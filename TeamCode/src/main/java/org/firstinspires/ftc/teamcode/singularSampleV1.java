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

@TeleOp(name = "Singular Samples", group = "Testing")
public class singularSampleV1 extends LinearOpMode {
    private OpenCvWebcam camera;
    private boolean cameraOpened = false;

    @Override
    public void runOpMode() {
        int viewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, viewId);

        camera.setPipeline(new ColorClassificationPipeline());
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
            telemetry.update();
        }
        camera.stopStreaming();
    }

    private static class ColorClassificationPipeline extends OpenCvPipeline {
        private Mat hsv = new Mat();
        private Mat mask = new Mat();
        private Mat hierarchy = new Mat();
        private List<MatOfPoint> contours = new ArrayList<>();

        // Calibration: 2 pixels per mm
        private static final double PIXELS_PER_MM = 2.0;
        private static final double MIN_SIZE_MM = 33.0;
        private static final double MAX_SIZE_MM = 88.0;
        private static final double EXPECTED_WIDTH_PX = MAX_SIZE_MM * PIXELS_PER_MM;
        private static final double EXPECTED_HEIGHT_PX = MAX_SIZE_MM * PIXELS_PER_MM;
        private static final double MAX_AREA_PX = EXPECTED_WIDTH_PX * EXPECTED_HEIGHT_PX * 1.5;
        private static final int MIN_AREA_PX = 200;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // combine all masks (example: red + blue)
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), mask);

            // morphology small kernel to clean noise, not merge
            Mat morphed = new Mat();
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);

            Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // draw on copy
            Mat output = input.clone();
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < MIN_AREA_PX || area > MAX_AREA_PX) continue;

                RotatedRect rRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                double widthMM = rRect.size.width / PIXELS_PER_MM;
                double heightMM = rRect.size.height / PIXELS_PER_MM;
                if (widthMM < MIN_SIZE_MM || widthMM > MAX_SIZE_MM || heightMM < MIN_SIZE_MM || heightMM > MAX_SIZE_MM)
                    continue;

                Point[] vertices = new Point[4];
                rRect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(output, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }
                String dims = String.format(Locale.ENGLISH, "%.1fmm x %.1fmm", widthMM, heightMM);
                Imgproc.putText(output, dims, rRect.center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6,
                        new Scalar(255, 255, 255), 2);
            }

            morphed.release(); kernel.release();
            return output;
        }
    }
}
