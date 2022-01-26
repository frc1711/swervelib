// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.util;

/**
 * A vector class used by {@link frc.team1711.swerve.subsystems.SwerveDrive} for calculating target velocities and rotational positions.
 * @author Gabriel Seaver
 */
public class Vector {
    
    private double x, y;
    
    /**
     * Makes a new vector.
     * @param x    The x component of the vector
     * @param y    The y component of the vector
     */
    public Vector (double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    /**
     * @return The x component of the vector
     */
    public double getX () {
        return x;
    }
    
    /**
     * @return The y component of the vector
     */
    public double getY () {
        return y;
    }
    
    /**
     * Rotates this vector to a given rotation.
     * @param radians   The rotation of the new vector, in radians, progressing
     * counterclockwise starting directly to the right of the origin on the x
     * axis.
     * @return          The new rotated vector
     */
    public Vector toRotationRadians (double radians) {
        final double mag = getMagnitude();
        return new Vector(mag * Math.cos(radians), mag * Math.sin(radians));
    }
    
    /**
     * Rotates this vector to a given rotation.
     * @param degrees   The rotation of the new vector, in degrees, progressing
     * clockwise starting directly above the origin on the y axis.
     * @return          The new rotated vector
     */
    public Vector toRotationDegrees (double degrees) {
        return toRotationRadians(Angles.degreesToRadians(degrees));
    }
    
    /**
     * Creates a new vector with the specified {@code direction} and {@code magnitude}.
     * @param direction The direction of the vector, in degrees, starting directly above
     * the origin on the y axis and progressing clockwise.
     * @param magnitude The magnitude of the vector
     * @return          The vector
     */
    public static Vector fromPolarDegrees (double direction, double magnitude) {
        return new Vector(1, 0).toRotationDegrees(direction).scale(magnitude);
    }
    
    /**
     * Creates a new vector with the specified {@code direction} and {@code magnitude}.
     * @param direction The direction of the vector, in radians, starting
     * directly right of the origin on the x axis and progressing counterclockwise
     * @param magnitude The magnitude of the vector
     * @return          The vector
     */
    public static Vector fromPolarRadians (double direction, double magnitude) {
        return new Vector(1, 0).toRotationRadians(direction).scale(magnitude);
    }
    
    /**
     * Adds a vector and returns the summed vector.
     * @param v2    The vector to add
     * @return      The summed vector
     */
    public Vector add (Vector v2) {
        return new Vector(x + v2.x, y + v2.y);
    }
    
    /**
     * Scales this vector and returns the resulting vector.
     * @param s     The scalar
     * @return      The scaled vector
     */
    public Vector scale (double s) {
        return new Vector(x * s, y * s);
    }
    
    /**
     * Reflects across the x axis.
     * @return      The result of reflecting this vector across the x axis.
     */
    public Vector reflectAcrossX () {
        return new Vector(x, -y);
    }
    
    /**
     * Reflects across the y axis.
     * @return      The result of reflecting this vector across the y axis.
     */
    public Vector reflectAcrossY () {
        return new Vector(-x, y);
    }
    
    /**
     * @return The counterclockwise rotation, in radians, with directly right of the origin
     * having a rotation of 0, in the range [0, 2pi).
     */
    public double getRotationRadians () {
        // Puts the coordinate on the unit circle, returning a rotation of 0 is the point is on the origin
        final double dist = Math.sqrt(x*x + y*y);
        if (dist == 0) return 0;
        
        // Math.max and min ensure the two values are within [-1, 1] (they may not be initially due to floating point error)
        double _x = Math.min(Math.max(x / dist, -1), 1);
        double _y = Math.min(Math.max(y / dist, -1), 1);
        
        // Finding the acos rotation
        double rotation = Math.acos(_x);
        
        // Adjusting for y value
        if (_y < 0) rotation = 2 * Angles.PI - rotation; // Acos can only find rotation based on x value, y is ignored
        
        return Angles.wrapRadians(rotation);
    }
    
    /**
     * @return The clockwise rotation, in degrees, with directly above the origin
     * having a rotation of 0, in the range [0, 360).
     */
    public double getRotationDegrees () {
        return Angles.radiansToDegrees(getRotationRadians());
    }
    
    /**
     * @return The magnitude of the vector
     */
    public double getMagnitude () {
        return Math.sqrt(x*x + y*y);
    }
    
    @Override
    public String toString () {
        final double _x = (double)(int)(x * 100) / 100;
        final double _y = (double)(int)(y * 100) / 100;
        final double _deg = (int)(getRotationDegrees());
        return "X: " + _x + ", Y: " + _y + ", Theta: " + _deg;
    }
    
}