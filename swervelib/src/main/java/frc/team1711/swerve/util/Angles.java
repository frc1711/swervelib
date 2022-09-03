// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.util;

/**
 * Provides useful methods allowing for more easily working with different
 * measurements of angles, along with the circle constants {@link #PI} and
 * {@link #TAU}.
 * @author Gabriel Seaver
 */
public class Angles {
    
    /**
     * The circle constant pi; the ratio from a circle's circumference to
     * its diameter. Equal to one half of {@link #TAU}. Also equal to the number
     * of radians in half a revolution.
     */
    public static double PI = Math.PI;
    
    /**
     * The circle constant tau; the ratio from a circle's circumference to
     * its radius. Equal to {@link #PI} times two. Also equal to the number
     * of radians in a complete revolution.
     */
    public static double TAU = PI * 2;
    
    /**
     * Wraps {@code degrees} to be on the interval [0, 360).
     * @param degrees   The unwrapped degrees
     * @return          The degrees, wrapped between 0 and 360
     */
    public static double wrapDegrees (double degrees) {
        while (degrees >= 360) degrees -= 360;
        while (degrees < 0) degrees += 360;
        return degrees;
    }
    
    /**
     * Wraps {@code degrees} to be on the interval [-180, 180).
     * @param degrees   The unwrapped degrees
     * @return          The degrees, wrapped between -180 and 180
     */
    public static double wrapDegreesZeroCenter (double degrees) {
        while (degrees >= 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }
    
    /**
     * Wraps {@code radians} to be on the interval [0, 2pi).
     * @param radians   The unwrapped radians
     * @return          The radians, wrapped between 0 and 2pi
     */
    public static double wrapRadians (double radians) {
        while (radians >= TAU) radians -= TAU;
        while (radians < 0) radians += TAU;
        return radians;
    }
    
    /**
     * Wraps {@code radians} to be on the interval [-pi, pi).
     * @param radians   The unwrapped radians
     * @return          The radians, wrapped between -pi and pi
     */
    public static double wrapRadiansZeroCenter (double radians) {
        while (radians >= PI) radians -= TAU;
        while (radians < -PI) radians += TAU;
        return radians;
    }
    
    /**
     * Converts a rotation in radians to degrees, where the radians starts at
     * directly right of the origin on the x axis and progresses counterclockwise,
     * and degrees starts directly above the origin on the y axis and progresses
     * clockwise.
     * @param radians   The radians to convert to degrees
     * @return          The degrees converted from radians
     */
    public static double radiansToDegrees (double radians) {
        double degrees = radians / PI * 180;
        degrees = -degrees; // Converts from counterclockwise to clockwise
        degrees += 90; // Converts from starting at right-pointing x axis to top-pointing y axis
        return degrees;
    }
    
    /**
     * Converts a rotation in degrees to radians, where the radians starts
     * directly right of the origin on the x axis and progresses counterclockwise,
     * and degrees starts directly above the origin on the y axis and progresses
     * clockwise.
     * @param degrees   The degrees to convert to radians
     * @return          The radians converted from degrees
     */
    public static double degreesToRadians (double degrees) {
        double radians = degrees * PI / 180;
        radians = -radians; // Converts from counterclockwise to clockwise
        radians += PI / 2; // Converts from starting at right-pointing x axis to top-pointing y axis
        return radians;
    }
    
}