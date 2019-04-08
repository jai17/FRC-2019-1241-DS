package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * 
 * Main Pipeline in which HSV Threshold and Contours are calculated Only Values
 * to be changed is Minimum Contours area HSV Threshold inputs are to be inputed
 * in *process* command in subsystem Implemented in Subsystem to be controlled
 * 
 * @author RickHansenRobotics
 * @since 2019-02-06
 *
 */
public class VisionPipeline {

	// Outputs
	private Mat hsvThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();

	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint2f> filterContoursOutput2f = new ArrayList<MatOfPoint2f>();

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the
	 * outputs.
	 */
	public void process(Mat source0, double[] hsvThresholdHue, double[] hsvThresholdSaturation,
			double[] hsvThresholdValue) {
		// Step HSV_Threshold0:
		Mat hsvThresholdInput = source0;
		hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Step Find_Contours0:
		Mat findContoursInput = hsvThresholdOutput;
		boolean findContoursExternalOnly = true;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;

		double filterContoursMinArea = 15.0; //15
		double filterContoursMinPerimeter = 0;
		double filterContoursMinWidth = 0;
		double filterContoursMaxWidth = 1000;
		double filterContoursMinHeight = 0;
		double filterContoursMaxHeight = 70;
		double[] filterContoursSolidity = { 0, 100 };
		double filterContoursMaxVertices = 1000000;
		double filterContoursMinVertices = 0;
		double filterContoursMinRatio = 0;
		double filterContoursMaxRatio = 1000;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter,
				filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight,
				filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio,
				filterContoursMaxRatio, filterContoursOutput, filterContoursOutput2f);

	} //end process

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * 
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThresholdOutput() {
		return hsvThresholdOutput;
	} //end hsvThresholdOutput

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * 
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	} //end findContoursOutput

	/**
	 * This method is a generated getter for the output of a Filter_Contours.
	 * 
	 * @return ArrayList<MatOfPoint> output from Filter_Contours.
	 */
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	} //end filterContoursOutput

	public ArrayList<MatOfPoint2f> filterContoursOutput2f() {
		return filterContoursOutput2f;
	} //end filterContoursOutput2f

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input  The image on which to perform the HSL threshold.
	 * @param hue    The min and max hue
	 * @param sat    The min and max saturation
	 * @param val    The min and max value
	 * @param output The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]), new Scalar(hue[1], sat[1], val[1]), out);
	} //end hsvThreshold

	/**
	 * Sets the values of pixels in a binary image to their distance to the nearest
	 * black pixel.
	 * 
	 * @param input    The image on which to perform the Distance Transform.
	 * @param type     The Transform.
	 * @param maskSize the size of the mask.
	 * @param output   The image in which to store the output.
	 */
	private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		} else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	} //end findContours

	/**
	 * Filters out contours that do not meet certain criteria.
	 * 
	 * @param inputContours  is the input list of contours
	 * @param output         is the the output list of contours
	 * @param minArea        is the minimum area of a contour that will be kept
	 * @param minPerimeter   is the minimum perimeter of a contour that will be kept
	 * @param minWidth       minimum width of a contour
	 * @param maxWidth       maximum width
	 * @param minHeight      minimum height
	 * @param maxHeight      maximimum height
	 * @param Solidity       the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio       minimum ratio of width to height
	 * @param maxRatio       maximum ratio of width to height
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea, double minPerimeter, double minWidth,
			double maxWidth, double minHeight, double maxHeight, double[] solidity, double maxVertexCount,
			double minVertexCount, double minRatio, double maxRatio, List<MatOfPoint> output,
			List<MatOfPoint2f> output2f) {
		final MatOfInt hull = new MatOfInt();
		output.clear();
		output2f.clear();
		
		// operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) //if outside of width range
				continue;
			if (bb.height < minHeight || bb.height > maxHeight) //if outside of height range
				continue;
			final double area = Imgproc.contourArea(contour); 
			if (area < minArea) //if smaller than minimum area
				continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) //if less than min perimeter
				continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int) hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1] };
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1])
				continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)
				continue;
			final double ratio = bb.width / (double) bb.height;
			if (ratio < minRatio || ratio > maxRatio)
				continue;

			output.add(contour);
			MatOfPoint2f NewMtx = new MatOfPoint2f(contour.toArray());
			output2f.add(NewMtx);
		}
	} //end filter contours

	/**	
	 * Calculates the direction a RotatedRect is slanted 
	 * @param r - RotatedRect to find the slope of
	 * @return -1 if angled right, 1 if angled left
	 */
	public double calcSlope(RotatedRect r) {
		//points fill bottomLeft, topLeft, topRight, bottomRight
		Point[] points = new Point[4];
		r.points(points);

		//calculate slope
		double y2 = 240 - points[2].y;
		double y1 = 240 - points[0].y;

		double x2 = points[2].x;
		double x1 = points[0].x;
		double slope = ((y2 - y1) / (x2 - x1));

		//return sign of slope 
		return Math.signum(slope);
	} //end calcSlope
}
