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
        private Mat morphed = new Mat(), hierarchy = new Mat(), labelMap = new Mat();
        private List<MatOfPoint> contours = new ArrayList<>();

        // Calibration: 2 pixels per mm
        private static final double PIXELS_PER_MM = 2.0;
        private static final double MIN_SIZE_MM = 33.0;
        private static final double MAX_SIZE_MM = 88.0;
        private static final double MAX_AREA_PX = (MAX_SIZE_MM*PIXELS_PER_MM)*(MAX_SIZE_MM*PIXELS_PER_MM)*1.5;
        private static final double MIN_AREA_PX = 200;
        private static final int KERNEL_SIZE = 3;
        private static final int TOLERANCE_PX = 20;

        @Override
        public Mat processFrame(Mat input) {
            // Prepare label map
            if (labelMap.empty() || !labelMap.size().equals(input.size())) {
                labelMap = Mat.zeros(input.size(), CvType.CV_8UC3);
            } else {
                labelMap.setTo(Scalar.all(0));
            }
            input.copyTo(labelMap);

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Red mask
            Core.inRange(hsv, new Scalar(0,100,100), new Scalar(10,255,255), maskRed1);
            Core.inRange(hsv, new Scalar(160,100,100), new Scalar(179,255,255), maskRed2);
            Core.bitwise_or(maskRed1, maskRed2, maskRed1);
            // Blue mask
            Core.inRange(hsv, new Scalar(100,100,100), new Scalar(130,255,255), maskBlue);

            // Process each color
            processColor(maskRed1, new Scalar(255,0,0));
            processColor(maskBlue, new Scalar(0,0,255));

            return labelMap;
        }

        private void processColor(Mat mask, Scalar drawColor) {
            // Morphology
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(KERNEL_SIZE,KERNEL_SIZE));
            Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);

            Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint cnt : contours) {
                double area = Imgproc.contourArea(cnt);
                if (area < MIN_AREA_PX || area > MAX_AREA_PX) continue;

                RotatedRect r = Imgproc.minAreaRect(new MatOfPoint2f(cnt.toArray()));
                double wMM = r.size.width/PIXELS_PER_MM;
                double hMM = r.size.height/PIXELS_PER_MM;
                if (wMM < MIN_SIZE_MM || wMM > MAX_SIZE_MM || hMM < MIN_SIZE_MM || hMM > MAX_SIZE_MM) continue;

                // Draw rotated box
                Point[] pts = new Point[4]; r.points(pts);
                for(int i=0;i<4;i++) Imgproc.line(labelMap, pts[i], pts[(i+1)%4], drawColor, 2);

                // Annotate size
                String dims = String.format(Locale.ENGLISH,"%.1fmm x %.1fmm", wMM,hMM);
                Imgproc.putText(labelMap, dims, r.center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,255,255),1);

                // Highlight if near target size
                if (Math.abs(r.size.width-(MAX_SIZE_MM*PIXELS_PER_MM)) <= TOLERANCE_PX &&
                        Math.abs(r.size.height-(MIN_SIZE_MM*PIXELS_PER_MM)) <= TOLERANCE_PX) {
                    for(int i=0;i<4;i++) Imgproc.line(labelMap, pts[i], pts[(i+1)%4], new Scalar(0,255,0),3);
                }
            }
            kernel.release(); morphed.release();
        }
    }
}
