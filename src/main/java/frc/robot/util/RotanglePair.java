package frc.robot.util;

import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

/**
 * RotanglePair
 * Created by: Neil Balaskandarajah
 * Last modified: 03/09/2019
 * Stores a pair of rotangles, representing a scoring/feeding location
 */

 public class RotanglePair {
    RotatedRect r1;
    RotatedRect r2;

    public RotanglePair(RotatedRect r1, RotatedRect r2) {
        this.r1 = r1;
        this.r2 = r2;
    }

    public double getCenterX() {
        return (r1.center.x + r2.center.x) / 2;
    }

    public double getCenterY() {
        return (r1.center.y + r2.center.y) / 2;
    }

    public Point getCenterPoint() {
        return new Point(getCenterX(), getCenterY());
    }

    public RotatedRect getLeftRotangle() {
        if (r1.center.x < r2.center.x) {
            return r1;
        } else {
            return r2;
        }
    }

    public RotatedRect getRightRotangle() {
        if (r1.center.x > r2.center.x) {
            return r1;
        } else {
            return r2;
        }
    }

 } //end class