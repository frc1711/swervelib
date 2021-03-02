// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An abstract class used by {@link SwerveDrive} to represent a module. Each {@code SwerveWheel}
 * contains a wheel which can rotate in any direction and drive forwards or backwards.
 * <b>Note: Based on encoder functionality, the wheel should be facing directly forwards when
 * this subsystem is instantiated, or {@link #resetEncoder()} should be used along with
 * a homing sequence.</b>
 * @author Gabriel Seaver
 */
abstract public class SwerveWheel extends SubsystemBase {
    
    /**
     * @return The wheel's positional rotation in revolutions, starting with 0 = directly forwards
     * and increasing as the wheel rotates clockwise (from a top-down point of view). The output
     * <b>should not be trimmed to stay within a certain range</b> (i.e. an output of 1.2 rotations
     * should not be looped around to the equivalent angle 0.2 rotations).
     */
    abstract protected double getRotation ();
    
    /**
     * This should set the target positional rotation of the wheel, using the same rotational
     * measurement as {@link #getRotation()}. This means that if you were to pass 1.2 as an
     * argument, the input <b>should not wrap to the equivalent angle of 0.2</b>, and the module
     * should instead perform whatever rotational action is necessary in order to make the
     * output of {@code getRotation()} closer to 1.2.
     * @param targetRotation The target rotation
     * @see #setSpeedAndRotation(double, double)
     * @see #setSpeed(double)
     */
    abstract protected void setRotation (double targetRotation);
    
    /**
     * Sets the directional speed of the wheel on the interval [-1, 1].
     * @param speed The direction speed
     * @see #setSpeedAndRotation(double, double)
     * @see #setRotation(double)
     */
    abstract protected void setSpeed (double speed);
    
    /**
     * Immediately stops all rotational movement.
     */
    abstract protected void stopRotationalMovement ();
    
    /**
     * Resets the encoders to a position of zero. This should only be used when
     * it is certain that the wheel is facing directly forward (i.e. the encoders
     * are properly set up such that {@link #getRotation()} will return the correct
     * rotation when called).
     */
    abstract public void resetEncoder ();
    
    /**
     * Sets the directional speed of the wheel, along with the target rotation
     * position of the module. The speed of the wheel should be on the interval
     * [-1, 1], and the target rotation should be on the interval [0, 360). A
     * target rotation of 0 degrees should correspond with moving directly forward,
     * where an increase in target rotation corresponds with a clockwise movement
     * in target rotational position (from a top-down point of view).
     * @param targetRot The target rotational position, as specified above
     * @param speed     The directional speed of the wheel
     * @see #setSpeed(double)
     * @see #setRotation(double)
     */
    public final void setSpeedAndRotation (double targetRot, double speed) {
        if (speed < -1 || speed > 1) throw new IllegalArgumentException("speed should be within range [-1, 1]");
        if (targetRot >= 360 || targetRot < 0) throw new IllegalArgumentException("targetRot should be within range [0, 360)");
        
        // Finds current rotation in degrees within range [0, 360)
        double currentRot = getRotation() * 360;
        while (currentRot < 0) currentRot += 360;
        while (currentRot >= 360) currentRot -= 360;
        
        // Finds the number of degrees we need to turn, plus the direction
        double moveRot = targetRot - currentRot;
        while (moveRot > 180) moveRot -= 360;
        while (moveRot < -180) moveRot += 360;
        
        // If the number of degrees we need to turn is closer
        // to 180 than 0, we turn the opposite way and go in reverse
        int reverse = 1;
        if (Math.abs(moveRot) > 90) {
            reverse = -1;
            if (moveRot > 0) moveRot -= 180;
            else moveRot += 180;
        }
        
        setRotation(moveRot / 360 + getRotation());
        setSpeed(speed * reverse);
    }
    
    /**
     * Sets directional speed to 0 and 
     */
    public final void stop () {
        setSpeed(0);
        stopRotationalMovement();
    }
    
}