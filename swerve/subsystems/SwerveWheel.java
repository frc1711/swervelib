// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swerve.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An abstract class used by {@link swerve.drive.SwerveDrive} to represent a module. Each {@code SwerveWheel}
 * contains a wheel which can steer in any direction and drive forwards or backwards.
 * <b>Note: Based on encoder functionality, the wheel should be facing directly forwards when
 * this subsystem is instantiated, or {@link #resetSteerEncoder()} should be used along with
 * a homing sequence.</b>
 * @author Gabriel Seaver
 */
abstract public class SwerveWheel extends SubsystemBase {
    
    /**
     * Creates a new {@code SwerveWheel}.
     */
    public SwerveWheel () {
        resetSteerEncoder();
    }
    
    /**
     * Gets the wheel's steering direction in revolutions, starting with 0 = directly forwards
     * and increasing as the wheel steers clockwise (from a top-down point of view). The output
     * <b>should not be trimmed to stay within a certain range</b> (i.e. an output of 1.2 revolutions
     * should not be looped around to the equivalent angle 0.2 revolutions).
     * @return The wheel's steering direction
     */
    abstract public double getDirection ();
    
    /**
     * Sets the target steering direction of the wheel, using the same rotational
     * measurement as {@link #getDirection()}. This means that if you were to pass 1.2 revolutions
     * as an argument, the input <b>should not wrap to the equivalent angle of 0.2 revolutions</b>,
     * and the module should instead perform whatever steering action is necessary in order to make the
     * output of {@code getDirection()} closer to 1.2.
     * @param targetDirection The target steering direction
     * @see #steerAndDrive(double, double)
     * @see #setDriveSpeed(double)
     */
    abstract public void setDirection (double targetDirection);
    
    /**
     * Sets the drive speed of the wheel on the interval [-1, 1].
     * @param speed The drive speed
     * @see #steerAndDrive(double, double)
     * @see #setDirection(double)
     */
    abstract public void setDriveSpeed (double speed);
    
    /**
     * Immediately stops all steering movement.
     */
    abstract public void stopSteering ();
    
    /**
     * Resets the steering encoder to a value of zero. This should only be used when
     * it is certain that the wheel is facing directly forward (i.e. the encoders
     * are properly set up such that {@link #getDirection()} will return the correct
     * steering direction when called).
     */
    abstract public void resetSteerEncoder ();
    
    /**
     * Sets the drive speed of the wheel, along with the target steering direction
     * of the module. The drive speed of the wheel should be on the interval [0, 1],
     * and the target steering direction should be on the interval [0, 360). A target
     * steering direction of 0 degrees should correspond with moving directly forward,
     * where an increase in target steering direction corresponds with a clockwise movement
     * in target steering direction (from a top-down point of view).
     * @param targetDirection   The target steering direction, as specified above
     * @param speed             The drive speed of the wheel
     * @see #setDriveSpeed(double)
     * @see #setDirection(double)
     */
    public final void steerAndDrive (double targetDirection, double speed) {
        if (speed < 0 || speed > 1) throw new IllegalArgumentException("speed should be within range [-1, 1]");
        if (targetDirection >= 360 || targetDirection < 0) throw new IllegalArgumentException("targetDirection should be within range [0, 360)");
        
        // Finds current direction in degrees within range [0, 360)
        double currentDirection = getDirection() * 360;
        while (currentDirection < 0) currentDirection += 360;
        while (currentDirection >= 360) currentDirection -= 360;
        
        // Finds the number of degrees we need to turn, plus the direction
        double moveDirection = targetDirection - currentDirection;
        while (moveDirection > 180) moveDirection -= 360;
        while (moveDirection < -180) moveDirection += 360;
        
        // If the number of degrees we need to turn is closer
        // to 180 than 0, we turn the opposite way and go in reverse
        int reverse = 1;
        if (Math.abs(moveDirection) > 90) {
            reverse = -1;
            if (moveDirection > 0) moveDirection -= 180;
            else moveDirection += 180;
        }
        
        setDirection(moveDirection / 360 + getDirection());
        setDriveSpeed(speed * reverse);
    }
    
    /**
     * Stops all movement.
     */
    public void stop () {
        setDriveSpeed(0);
        stopSteering();
    }
    
}