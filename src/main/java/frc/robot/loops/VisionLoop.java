package frc.robot.loops;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Robot;
import frc.robot.util.VisionPipeline;

public class VisionLoop implements Loop {

    private static VisionLoop mInstance;

    Thread visionThread;
    VisionPipeline VisionPipeline;
    double avgX;
    double avgArea;

    public enum VisionControlState {
        VISION_LOOP, // open loop voltage control
    }

    private VisionControlState mControlState = VisionControlState.VISION_LOOP;

    public static VisionLoop getInstance() {
        if (mInstance == null) {
            mInstance = new VisionLoop();
            return mInstance;
        } else
            return mInstance;
    }

    private VisionLoop() {
        VisionPipeline = new VisionPipeline();

        /**
         * Initializaes a new VisionThread in which the VisionPipeline is implemented in
         * order to input param for: HSV HUE, SAT, VAL CAMERA SERVER INPUT, RESOLUTION,
         * FPS SETTING THE OUTPUT STREAM TO BE EITHER HSV, CONTOURS, BOUNDRECT SETTING
         * BOUNDRECT AROUND CONTOURS
         */
        visionThread = new Thread(() -> {
            Mat contours = new Mat(); /** MOST LIKELY UNECESSARY **/

            /** Get the UsbCamera from CameraServer */
            UsbCamera camera = CameraServer .getInstance().startAutomaticCapture();

            /** Set the resolution */
            camera.setResolution(160, 120);
            camera.setExposureManual(-20);
            camera.setExposureHoldCurrent();
            camera.setBrightness(-20);

            /**
             * Set the FPS NOTE: Param Set will not necessarily be what is displayed
             */
            camera.setFPS(20);

            /** Get a CvSink. This will capture Matrixes from the camera */
            CvSink cvSink = CameraServer.getInstance().getVideo();

            /**
             * Setup a CvSource. This will send images back to the Dashboard NOTE: Called
             * Rectangle
             */
            CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 320, 240);

            /** Mats are very memory expensive. Lets reuse this Mat. */
            Mat mat = new Mat();
            Mat frame = new Mat();

            /**
             * This cannot be 'true'. The program will never exit if it is. This lets the
             * robot stop this thread when restarting robot code or deploying.
             */
            while (!Thread.interrupted()) {

                /**
                 * Tell the CvSink to grab a frame from the camera and put it in the source mat.
                 * If there is an error notify the output.
                 */
                if (cvSink.grabFrame(mat) == 0) {
                    // Send the output the error.
                    outputStream.notifyError(cvSink.getError());
                    // skip the rest of the current iteration
                    continue;
                }
                frame = mat;

                /**
                 * Provide the visionPipeline method *process* with the image source in our case
                 * mat as well as the hsvThresholds we provide from the Robot Class Array
                 * Preferences
                 */
                VisionPipeline.process(mat, Robot.hsvThresholdHue, Robot.hsvThresholdSat, Robot.hsvThresholdVal);
                /**
                 * Below is the Method(s) used to draw a rectangle around found contours For
                 * each additional rectangle, use >0, >1 etc. Requires: IMAGE, IN OUR CASE WILL
                 * ALWAYS BE mat FIRST POINT: WILL BE THE RECT.X AND RECT.Y SECOND POINT: WILL
                 * BE R.X + WIDTH AND R.Y + HEIGHT SCALAR: COLOR OF THE BOUNDING RECTANGLE (RED
                 * IS NICE) THICKNESS: THE THICKNESS OF THE BOUNDING RECTANGLE
                 */
                // System.out.println(filterContoursOutput.size()); USED TO
                // OUTPUT HOW MANY CONTOURS YOU HAVE TO BOUND
                if (VisionPipeline.filterContoursOutput().size() > 0) {
                    Rect r = Imgproc.boundingRect(VisionPipeline.filterContoursOutput().get(0));
                    Imgproc.rectangle(frame, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height),
                            new Scalar(0, 0, 255), 2);

                    avgX = r.x; // Takes the x cordinate of the first rectangle

                    if (VisionPipeline.filterContoursOutput().size() > 1) {
                        Rect r2 = Imgproc.boundingRect(VisionPipeline.filterContoursOutput().get(1));
                        Imgproc.rectangle(frame, new Point(r2.x, r2.y), new Point(r2.x + r.width, r2.y + r2.height),
                                new Scalar(0, 0, 255), 2);

                        avgX += r2.width + r2.x; // Adds the width and X
                                                 // cordinate of the second
                                                 // rectangle
                        avgX = avgX / 2; // Divides the Average X cordinate by
                                         // two for half of the image center
                        avgArea = (r.area() + r2.area()) / 2;
                    }
                    /**
                     * Outputs Bounding Rectangles if there are any or outputs hsv Threshold output
                     */
                    outputStream.putFrame(frame);
                } else {
                    outputStream.putFrame(VisionPipeline.hsvThresholdOutput());
                }
            }
        });
        visionThread.setDaemon(true);
        visionThread.start();

    }

    @Override
    public void onStart(double time_stamp) {
        System.out.println("Hatch Loop Started");
    }

    @Override
    public void onLoop(double time_stamp) {
        switch (mControlState) {
        case VISION_LOOP:

            return;
        }
    }

    @Override
    public void onStop(double time_stamp) {
        // TODO Auto-generated method stub
    }

    /** Stores the average of the XCordinates */
    public double avg() {
        return avgX;
    }

    public double avgArea() {
        return avgArea;
    }

    public double pixelToDegree(double pixel) {
        // return 0.0870234789*pixel-28.5146932592;
        return Math.toDegrees(Math.atan(((pixel - 160) * Math.tan(Math.toRadians(31.81))) / 160));
    }

    public void setVisionState(VisionControlState state) {
        mControlState = state;
    }

    public VisionControlState getControlState() {
        return mControlState;
    }

}