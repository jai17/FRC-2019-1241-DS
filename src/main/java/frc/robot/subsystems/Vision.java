/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

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
import frc.robot.util.RotanglePair;
import frc.robot.util.Rotangle;

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

    public VisionTrackingState mTrackingState = VisionTrackingState.NO_TARGET;

    public static Vision getInstance() {
        if (mInstance == null) {
            mInstance = new Vision();
            return mInstance;
        } else
            return mInstance;
    }

    //States for Vision to report to Dashboard
    public enum VisionTrackingState {
        CARGO_SHIP,
        ROCKET,
        NO_TARGET
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

                //number of contours on the screen
                int numContours = VisionPipeline.filterContoursOutput().size();
                
                /** More Than Two: Cargo Ship */
                if (numContours > 2) {
                    mTrackingState = VisionTrackingState.CARGO_SHIP;

                    //all rotangles
                    List<Rotangle> rotangles = new ArrayList<Rotangle>();

                    //Add all rotangles from filtered contours
                    for (int r = 0; r < numContours; r++) {
                        rotangles.add(new Rotangle(Imgproc.minAreaRect(VisionPipeline.filterContoursOutput2f().get(r))));
                    }

                    //sort rotangles from left to right
                    Collections.sort(rotangles);

                    //For Judges: Draw numbers associated to rotangles 
                    /*
                    int loop = 0;
                    for (Rotangle rot : rotangles) {
                        Imgproc.putText(frame, Integer.toString(loop), rot.getRect().center, 1, 2.0, new Scalar(0,0,255));
                        loop++;
                    }*/

                    //find all pairs
                    RotanglePair[] pairs = new RotanglePair[numContours-1];

                    for(int n = 0; n < (numContours-1); n++) {
                        pairs[n] = new RotanglePair(rotangles.get(n).getRect(), rotangles.get(n+1).getRect());
                        //For Judges: Draw numbers of paired rotangles
                        // Imgproc.putText(frame, Integer.toString(n), pairs[n].getCenterPoint(), 1, 1.0, new Scalar(0,255,0));
                    }

                    //find all targets
                    ArrayList<RotanglePair> targets = new ArrayList<RotanglePair>();

                    for (int t = 0; t < (numContours - 1); t++) {
                        if(VisionPipeline.calcSlope(pairs[t].getLeftRotangle()) == -1) { //if left is left sloped and right is right sloped
                            targets.add(pairs[t]);
                        } else {
                        }
                    }

                    //choose closest target
                    int wanted = 0; // index of desired pair of targets
                    double closestCenter = 0; // smallest difference from pair's center x to center

                    //find target
                    for (int i = 0; i < targets.size(); i++) {
                        if (i == 0) { // assign closest center first run
                            closestCenter = Math.abs(targets.get(i).getCenterX() - 80);
                        } else if ((targets.get(i).getCenterX() - 80) < closestCenter) { // if closer to center
                            wanted = i;
                        }
                    } //found pairs

                    //set average x and y
                    if (targets.size() != 0) {
                        avgX = targets.get(wanted).getCenterX();
                        avgY = targets.get(wanted).getCenterY();

                        //draw rotangles
                        Point[] leftVertices = new Point[4];
                        targets.get(wanted).getLeftRotangle().points(leftVertices);
    
                        Point[] rightVertices = new Point[4];
                        targets.get(wanted).getRightRotangle().points(rightVertices);
    
                        for (int z = 0; z < 4; z++) {
                            Imgproc.line(frame, leftVertices[z], leftVertices[(z + 1) % 4], new Scalar(0, 0, 255), 2); //Purple: (230, 31, 177)
                            Imgproc.line(frame, rightVertices[z], rightVertices[(z + 1) % 4], new Scalar(0, 0, 255), 2);
                        } //drew lines
                    }

                    outputStream.putFrame(frame);
                /** Less than 3 but greater than 0: Rocket Ship */
                } else if (numContours == 2) {
                    mTrackingState = VisionTrackingState.ROCKET;
                    
                    Rect rectLeft = Imgproc.boundingRect(VisionPipeline.filterContoursOutput().get(0)); 
                    Rect rectRight = Imgproc.boundingRect(VisionPipeline.filterContoursOutput().get(1));

                    avgX = (rectLeft.x + rectRight.x)/2; 
                    avgY  = (rectLeft.y + rectRight.y)/2;  

                    Imgproc.rectangle(frame, new Point (rectLeft.x, rectLeft.y), new Point (rectLeft.x + rectLeft.width, rectLeft.y + rectLeft.height), new Scalar(0, 0, 255), 2);
                    Imgproc.rectangle(frame, new Point (rectRight.x, rectRight.y), new Point (rectRight.x + rectRight.width, rectLeft.y + rectLeft.height), new Scalar(0, 0, 255), 2);

                    //Using the rotangle method for rocket mode
                    /*
                    //get both rotangles
                    RotatedRect rRot = Imgproc.minAreaRect(VisionPipeline.filterContoursOutput2f().get(0));
                    RotatedRect rRot2 = Imgproc.minAreaRect(VisionPipeline.filterContoursOutput2f().get(1));

                    //create target object
                    RotanglePair target = new RotanglePair(rRot, rRot2);
                    
                    //get average x and y
                    avgX = target.getCenterX();
                    avgY = target.getCenterY();

                    //fill rotangle points into arrays for drawing
                    Point[] leftVertices = new Point[4];
                    target.getLeftRotangle().points(leftVertices);

                    Point[] rightVertices = new Point[4];
                    target.getRightRotangle().points(rightVertices);

                    //draw rotangles
                    for(int d = 0; d < 4; d++) {
                        Imgproc.line(frame, leftVertices[d], leftVertices[(d+1) %4], new Scalar(255, 255, 0));
                        Imgproc.line(frame, rightVertices[d], rightVertices[(d+1) %4], new Scalar(255, 255, 0));                        
                    }*/

                    //put the frame to the screen
                    outputStream.putFrame(frame);
                    
                    /** ELSE: No Targets */
                } else {
                    mTrackingState = VisionTrackingState.NO_TARGET;
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
        return -Math.toDegrees(Math.atan(((pixel - 80) * Math.tan(Math.toRadians(28))) / 80));// 31.81
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

    /**
     * Returns the state of the vision tracking system
     * @return mTrackingState - state the vision system is in
     */
    public VisionTrackingState getTrackingState() {
        return mTrackingState;
    }
}
