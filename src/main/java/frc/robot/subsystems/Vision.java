/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.util.VisionPipeline;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {

    private static Vision mInstance;

    Thread visionThread;
    VisionPipeline VisionPipeline;
    double avgX;
    double avgY;
    double avgArea;
    public static final double CAMERA_ANGLE = 16.1;
    public double targetWidth;

    public static Vision getInstance() {
        if (mInstance == null) {
            mInstance = new Vision();
            return mInstance;
        } else
            return mInstance;
    }

    public Vision() {
        VisionPipeline = new VisionPipeline();

        /**
         * Initializaes a new VisionThread in which the VisionPipeline is implemented in
         * order to input param for: HSV HUE, SAT, VAL, CAMERA SERVER INPUT, RESOLUTION,
         * FPS SETTING THE OUTPUT STREAM TO BE EITHER HSV, CONTOURS, BOUNDRECT SETTING
         * BOUNDRECT AROUND CONTOURS
         */
        visionThread = new Thread(() -> {
            Mat contours = new Mat(); /** MOST LIKELY UNECESSARY **/

            VideoSink server;

            /** Get the UsbCamera from CameraServer */
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
            UsbCamera carriageCamera = CameraServer.getInstance().startAutomaticCapture(1);
            // server = CameraServer.getInstance().getServer();

            // if (Robot.m_oi.getDriveRightTrigger()){
            // server.setSource(camera);
            // } else if (Robot.m_oi.getDriveLeftTrigger()) {
            // server.setSource(carriageCamera);
            // }

            /** Set the resolution */
            camera.setResolution(160, 120);
            // camera.setExposureAuto()
            // camera.setWhiteBalanceAuto();
            camera.setExposureManual(-10);
            camera.setExposureHoldCurrent();
            camera.setWhiteBalanceAuto();
            camera.setBrightness(-10);

            // carriage camera settings
            carriageCamera.setResolution(160, 120);
            carriageCamera.setExposureAuto();
            carriageCamera.setFPS(15);
            carriageCamera.setBrightness(60);

            /**
             * Set the FPS NOTE: Param Set will not necessarily be what is displayed
             */
            camera.setFPS(15);
            camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

            /** Get a CvSink. This will capture Matrixes from the camera */
            CvSink cvSink = CameraServer.getInstance().getVideo();

            /**
             * Setup a CvSource. This will send images back to the Dashboard NOTE: Called
             * "Rectangle" on Dashboard
             */
            CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 160, 120);

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

                /** More Than Two: Cargo Ship */
                if (VisionPipeline.filterContoursOutput().size() > 2) {

                    // as many rotangles as there are contours
                    RotatedRect[] rotangles = new RotatedRect[VisionPipeline.filterContoursOutput().size()];

                    RotatedRect[] lefts = new RotatedRect[6];
                    RotatedRect[] rights = new RotatedRect[6];
                    int leftIndex = 0;
                    int rightIndex = 0;

                    System.out.println("created rotangle arrays");

                    // pump rotangles into array
                    for (int i = 0; i < VisionPipeline.filterContoursOutput().size(); i++) {
                        rotangles[i] = Imgproc.minAreaRect(VisionPipeline.filterContoursOutput2f().get(i));

                        // Rect r = Imgproc.boundingRect(VisionPipeline.filterContoursOutput().get(0));
                        // RotatedRect rRotangle =
                        // Imgproc.minAreaRect(VisionPipeline.filterContoursOutput2f().get(i));
                        // Imgproc.rectangle(frame, new Point(r.x, r.y), new Point(r.x + r.width, r.y +
                        // r.height),
                        // new Scalar(0, 0, 255), 2);
                        double angle = rotangles[i].angle;
                        System.out.println(i + " Angle: " + angle);

                        // angle ranges
                        double minRight = -60;
                        double maxRight = -90;
                        double minLeft = -0;
                        double maxLeft = -30;

                        // save wanted rotangles into respective left and right arrays
                        if (angle >= maxLeft && angle <= minLeft) { // if left angled rectangle
                            
                            lefts[leftIndex] = rotangles[i];
                            leftIndex++;
                            System.out.println("saved left");
                        } else if (angle >= maxRight && angle <= minRight && (i != 0)) { // if right angled rectangle
                            if (rightIndex >= leftIndex) {
                                lefts[leftIndex] = rotangles[i];
                                leftIndex++;
                            } else {
                                rights[rightIndex] = rotangles[i];
                                rightIndex++;
                            }
                            System.out.println("saved right");
                        }
                    } // rotangles
                    System.out.println("Saved pairs of rotangles");

                    double[] centers = new double[rightIndex]; // center positions from each pairs
                    int kevinSpacey = 0; // index of desired pair of targets
                    double closestCenter = 0; // smallest difference from pair's center x to center (80 pixels)

                    // find pairs
                    for (int i = 0; i < centers.length; i++) {
                        centers[i] = (lefts[i].center.x + rights[i].center.x) / 2;

                        if (i == 0) { // assign closest center first run
                            closestCenter = Math.abs(centers[i] - 80);
                        } else if (Math.abs(centers[i] - 80) < closestCenter) { // if closer to center
                            kevinSpacey = i;
                        }
                    }
                    System.out.println("found pairs of rotangles");

                    // DRAWING ROTANGLES
                    Point[] leftVertices = new Point[4];
                    lefts[kevinSpacey].points(leftVertices);

                    Point[] rightVertices = new Point[4];
                    rights[kevinSpacey].points(rightVertices);

                    // draw pair of lines
                    for (int z = 0; z < 4; z++) {
                        Imgproc.line(frame, leftVertices[z], leftVertices[(z + 1) % 4], new Scalar(230, 31, 177), 2);
                        Imgproc.line(frame, rightVertices[z], rightVertices[(z + 1) % 4], new Scalar(230, 31, 177), 2);
                    }

                    System.out.println("Drew rotangles");

                    // assign center points
                    avgX = (lefts[kevinSpacey].center.x + rights[kevinSpacey].center.x) / 2;
                    avgY = (lefts[kevinSpacey].center.y + rights[kevinSpacey].center.y) / 2;

                    // put frame
                    outputStream.putFrame(frame);
                    System.out.println("put frame");

                    /** Less than 3 but greater than 0: Rocket Ship */
                } else if (VisionPipeline.filterContoursOutput().size() == 2) {
                    // if (VisionPipeline.filterContoursOutput().size() > 0) {
                    // Rect r = Imgproc.boundingRect(VisionPipeline.filterContoursOutput().get(0));
                    RotatedRect rRot = Imgproc.minAreaRect(VisionPipeline.filterContoursOutput2f().get(0));
                    System.out.println("left rotangle " + rRot.angle);
                    // Imgproc.rectangle(frame, new Point(r.x, r.y), new Point(r.x + r.width, r.y +
                    // r.height),
                    // new Scalar(0, 0, 255), 2);
                    Point[] vertices = new Point[4];
                    rRot.points(vertices);

                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(frame, vertices[i], vertices[(i + 1) % 4], new Scalar(230, 31, 177), 2);
                    }

                    avgX = rRot.center.x; // Takes the x cordinate of the first rectangle
                    avgY = rRot.center.y; // Takes the y cordinate of the first rectangle

                    // if (VisionPipeline.filterContoursOutput().size() > 1) {
                    // Rect r2 = Imgproc.boundingRect(VisionPipeline.filterContoursOutput().get(1));
                    RotatedRect rRot2 = Imgproc.minAreaRect(VisionPipeline.filterContoursOutput2f().get(1));
                    System.out.println("right rotangle " + rRot2.angle);

                    // Imgproc.rectangle(frame, new Point(r2.x, r2.y), new Point(r2.x + r.width,
                    // r2.y + r2.height),
                    // new Scalar(0, 0, 255), 2);

                    Point[] vertices2 = new Point[4];
                    rRot2.points(vertices2);

                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(frame, vertices2[i], vertices2[(i + 1) % 4], new Scalar(230, 31, 177), 2); // bgr
                    }

                    // avgX += r2.width + r2.x; // Adds the width and X
                    // // cordinate of the second
                    // // rectangle
                    // avgY += r2.height + r2.y; // Adds the height and Y
                    // // cordinate of the second
                    // // rectangle

                    avgX += rRot2.center.x; // Adds the width and X
                                            // cordinate of the second
                                            // rectangle
                    avgY += rRot2.center.y; // Adds the height and Y
                                            // cordinate of the second
                                            // rectangle

                    targetWidth = (rRot2.center.x + rRot2.size.width / 2) - (rRot.center.x - rRot.size.width / 2);
                    // System.out.println(targetWidth +" "+ this.toString());

                    avgX = avgX / 2; // Divides the Average X cordinate by
                                     // two for half of the image center
                    avgY = avgY / 2; // Divides the Average Y cordinate by
                                     // two for half of the image center

                    // avgArea = (r.area() + r2.area()) / 2;

                    /**
                     * Outputs Bounding Rectangles if there are any or outputs hsv Threshold output
                     */
                    outputStream.putFrame(frame);

                    /** ELSE: No Targets */
                } else {
                    outputStream.putFrame(VisionPipeline.hsvThresholdOutput());
                }
            }
        });
        visionThread.setDaemon(true);
        visionThread.start();

    }

    /** Stores the average of the XCordinates */
    public double avg() {
        return avgX;
    }

    public double avgY() {
        return avgY;
    }

    public double avgArea() {
        return avgArea;
    }

    public double pixelToDegree(double pixel) {
        // return 0.0870234789*pixel-28.5146932592;
        return -Math.toDegrees(Math.atan(((pixel - 80) * Math.tan(Math.toRadians(31.81))) / 80));// 31.81
    }

    public double pixelToDegreeY(double pixel) {
        // return 0.0870234789*pixel-28.5146932592;
        return Math.toDegrees(Math.atan(((pixel - 60) * Math.tan(Math.toRadians(31.81))) / 60));// 31.81
    }

    public double getCameraAngle(double angleToTargetY, double heightCamera, double heightTarget, double distance) {
        return Math.atan((heightTarget - heightCamera) / distance) - angleToTargetY;
    }

    /**
     * 
     * @param a1 - angle to the target from the camera
     * @param a2 - angle camera is angled
     * @param h1 - height of camera off the ground
     * @param h2 - height of target off the ground
     * @return horizontal distance from the target
     */
    // public double getDistance(double a1, double a2, double h1, double h2) {
    // // return (h2 - h1) / Math.tan(Math.toRadians(a1 + a2));
    // }

    public double getDistance(int fieldOfViewPixels, double halfLensFOV, double targetWidthPixels) {
        double targetWidth = 14;
        return targetWidth * fieldOfViewPixels / (2 * targetWidthPixels * Math.tan(halfLensFOV));
    }

    public double getWidth() {
        return targetWidth;
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
