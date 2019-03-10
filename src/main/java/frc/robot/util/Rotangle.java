package frc.robot.util;

import org.opencv.core.RotatedRect;

/**
 * Rotangle
 * Created by: Neil Balaskandarajah
 * Last modified: 03/09/2019
 * A comparable rotated rectangle
 */
public class Rotangle extends RotatedRect implements Comparable<Rotangle> {
    RotatedRect rRot;

    public Rotangle() {
        super();
        rRot = this;
    }

    public Rotangle(RotatedRect r) {
        rRot = r;
    }

    public RotatedRect getRect() {
        return rRot;
    }

    public int compareTo(Rotangle r) {
        if (getRect().center.x > r.getRect().center.x) {
            return 1;
        } else if (getRect().center.x < r.getRect().center.x) {
            return -1;
        } else {
            return 0;
        }
    }
} //end class