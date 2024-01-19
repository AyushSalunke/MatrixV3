package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.Globals.SIDE;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Config
public class PropPipeline implements VisionProcessor {
    private static final boolean DEBUG = true;

    private volatile Location location = Location.RIGHT;

    private final Mat hsv = new Mat();

    public static double Factor=0.55;



    public static int redLeftX = 750; //250;//(int) (815 / 1.5);
    public static int redLeftY = 265; //360;//(int) (550 / 1.5);

    public static int redCenterX = 500;//570;//(int) (1365 / 1.5);
    public static int redCenterY = 280; //360;//(int) (475 / 1.5);

    public static int blueLeftX = 940;//(int) (240 / 1.5);
    public static int blueLeftY = 255;//(int) (525 / 1.5);

    public static int blueCenterX = 650;
    public static int blueCenterY = 280;
    /////////////////////////////////////////////////
    public static int leftWidth =100; //(int) (175 / 1.5);
    public static int leftHeight = 70;//(int) (100 / 1.5);
    /////////////////////////////////////////////////
    public static int centerWidth = 60;//(int) (125 / 1.5);
    public static int centerHeight = 60;//(int) (125 / 1.5);
    /////////////////////////////////////////////////
    public static double BLUE_TRESHOLD = 100;
    public static double RED_TRESHOLD = 100;

    public double leftColor = 0.0;
    public double centerColor = 0.0;

    public Scalar left = new Scalar(0, 0, 0);
    public Scalar center = new Scalar(0, 0, 0);

    Telemetry telemetry;

    Location ALLIANCE = Globals.ALLIANCE;

    public PropPipeline() {
        this(null);
    }

    public PropPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Rect leftZoneArea;
        Rect centerZoneArea;

        if (ALLIANCE == Location.RED && SIDE == Location.FAR || ALLIANCE == Location.BLUE && SIDE == Location.CLOSE) {
            leftZoneArea = new Rect(blueLeftX, blueLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(blueCenterX, blueCenterY, centerWidth, centerHeight);
        } else {
            leftZoneArea = new Rect(redLeftX, redLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(redCenterX, redCenterY, centerWidth, centerHeight);
        }

        Mat leftZone = frame.submat(leftZoneArea);
        Mat centerZone = frame.submat(centerZoneArea);


        if (DEBUG) {
            Imgproc.blur(frame, frame, new Size(5, 5));
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 2);
            Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 2);
        }

        Imgproc.blur(leftZone, leftZone, new Size(5, 5));
        Imgproc.blur(centerZone, centerZone, new Size(5, 5));

        left = Core.mean(leftZone);
        center = Core.mean(centerZone);

        if (telemetry != null) {
            telemetry.addData("leftColor", left.toString());
            telemetry.addData("centerColor", center.toString());
            telemetry.addData("analysis", location.toString());
            telemetry.update();
        }

//        telemetry.addData("leftColor", left.toString());
//        telemetry.addData("centerColor", center.toString());
//        telemetry.addData("analysis", location.toString());
//        telemetry.update();

        double threshold = ALLIANCE == Location.RED ? RED_TRESHOLD : BLUE_TRESHOLD;
        int idx = ALLIANCE == Location.RED ? 0 : 2;

        leftColor = left.val[idx];
        centerColor = center.val[idx];

        if (leftColor > threshold && ((left.val[0] + left.val[1] + left.val[2] - left.val[idx])*Factor < left.val[idx])) {
            // left zone has it
            location = Location.LEFT;
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 10);
        } else if (centerColor > threshold && ((center.val[0] + center.val[1] + center.val[2] - center.val[idx])*Factor < center.val[idx])) {
            // center zone has it
            location = Location.CENTER;
            Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 10);
        } else {
            // right zone has it
            location = Location.RIGHT;
        }

        leftZone.release();
        centerZone.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Location getLocation() {
        return this.location;
    }
}