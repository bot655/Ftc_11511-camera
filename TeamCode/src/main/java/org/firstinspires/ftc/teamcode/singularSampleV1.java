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
        private Mat maskRed1 = new Mat(), maskRed2 = new Mat(), maskBlue = new Mat();
        private Mat morphed = new Mat(), hierarchy = new Mat(), output = new Mat();

        private static final double PIXELS_PER_MM = 2.0;
        private static final double MIN_SIZE_MM = 33.0;
        private static final double MAX_SIZE_MM = 88.0;
        private static final double MAX_AREA_PX = MAX_SIZE_MM * PIXELS_PER_MM * MAX_SIZE_MM * PIXELS_PER_MM * 1.5;
        private static final double MIN_AREA_PX = 200;
        private static final int KERNEL_SIZE = 3;
        private static final int TOLERANCE_PX = 20;

        @Override
        public Mat processFrame(Mat input) {
            // clone input for output
            input.copyTo(output);

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            // create color masks
            Core.inRange(hsv, new Scalar(0,100,100), new Scalar(10,255,255), maskRed1);
            Core.inRange(hsv, new Scalar(160,100,100), new Scalar(179,255,255), maskRed2);
            Core.bitwise_or(maskRed1, maskRed2, maskRed1);
            Core.inRange(hsv, new Scalar(100,100,100), new Scalar(130,255,255), maskBlue);

            // combine both masks
            Mat combined = new Mat();
            Core.bitwise_or(maskRed1, maskBlue, combined);

            // clean up noise but avoid merging
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(KERNEL_SIZE, KERNEL_SIZE));
            Imgproc.morphologyEx(combined, morphed, Imgproc.MORPH_OPEN, kernel);

            // find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // select best candidate
            RotatedRect bestRect = null;
            double bestScore = Double.MAX_VALUE;
            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area < MIN_AREA_PX || area > MAX_AREA_PX) continue;
                RotatedRect r = Imgproc.minAreaRect(new MatOfPoint2f(c.toArray()));
                double wMM = r.size.width / PIXELS_PER_MM;
                double hMM = r.size.height / PIXELS_PER_MM;
                if (wMM < MIN_SIZE_MM || wMM > MAX_SIZE_MM || hMM < MIN_SIZE_MM || hMM > MAX_SIZE_MM) continue;
                double score = Math.abs(r.size.width - MAX_SIZE_MM*PIXELS_PER_MM) + Math.abs(r.size.height - MIN_SIZE_MM*PIXELS_PER_MM);
                if (score < bestScore) {
                    bestScore = score;
                    bestRect = r;
                }
            }

            // draw only best
            if (bestRect != null) {
                Point[] pts = new Point[4]; bestRect.points(pts);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(output, pts[i], pts[(i+1)%4], new Scalar(0,255,0), 3);
                }
                String dims = String.format(Locale.ENGLISH, "%.1fmm x %.1fmm", bestRect.size.width/PIXELS_PER_MM, bestRect.size.height/PIXELS_PER_MM);
                Imgproc.putText(output, dims, bestRect.center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255,255,255), 2);
            }

            // release temporaries
            kernel.release(); morphed.release(); combined.release();
            return output;
        }
    }
}
