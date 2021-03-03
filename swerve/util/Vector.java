// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swerve.util;

/**
 * A vector class used by {@link swerve.drive.SwerveDrive} for calculating target velocities and rotational positions.
 * @author Gabriel Seaver
 */
public class Vector {
    
    private double x, y;
    
    /**
     * Makes a new vector.
     * @param _x    The x component of the vector
     * @param _y    The y component of the vector
     */
    public Vector (double _x, double _y) {
        x = _x;
        y = _y;
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
        final double TAU = 2 * Math.PI;
        
        // Puts the coordinate on the unit circle, returning a rotation of 0 is the point is on the origin
        final double dist = Math.sqrt(x*x + y*y);
        if (dist == 0) return 0;
        
        // Math.max and min ensure the two values are within [-1, 1] (they may not be initially due to floating point error)
        double _x = Math.min(Math.max(x / dist, -1), 1);
        double _y = Math.min(Math.max(y / dist, -1), 1);
        
        // Finding the acos rotation
        double rotation = Math.acos(_x);
        
        // Adjusting for y value
        if (_y < 0) rotation = 2 * Math.PI - rotation; // Acos can only find rotation based on x value, y is ignored
        
        // Puts rotation within bounds of [0, 2pi)
        while (rotation < 0) rotation += TAU;
        while (rotation >= TAU) rotation -= TAU;
        
        return rotation;
    }
    
    /**
     * @return The clockwise rotation, in degrees, with directly above the origin
     * having a rotation of 0, in the range [0, 360).
     */
    public double getRotationDegrees () {
        final double radians = getRotationRadians();
        double degrees = radians / Math.PI * 180;
        
        degrees = -degrees; // Converts from counterclockwise to clockwise
        degrees += 90; // Converts from starting at right-pointing x axis to top-pointing y axis
        
        // // Puts into interval [0, 360)
        while (degrees < 0) degrees += 360;
        while (degrees >= 360) degrees -= 360;
        
        return degrees;
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