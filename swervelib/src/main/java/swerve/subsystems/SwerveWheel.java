// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swerve.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An abstract class used by {@link swerve.drive.SwerveDrive} to represent a
 * module. Each {@code SwerveWheel} contains a wheel which can steer in any
 * direction and drive forwards or backwards. <b>Note: Based on encoder
 * functionality, the wheel should be facing directly forwards when this
 * subsystem is instantiated, or {@link #resetSteerEncoder()} should be used
 * along with a homing sequence.</b>
 * 
 * @author Gabriel Seaver
 */
abstract public class SwerveWheel extends SubsystemBase {
    
    /**
     * Gets the wheel's steering direction in revolutions, starting with 0 = directly forwards
     * and increasing as the wheel steers clockwise (from a top-down point of view). The output
     * <b>should not be trimmed to stay within a certain range</b> (i.e. an output of 1.2 revolutions
     * should not be looped around to the equivalent angle 0.2 revolutions).
     * @return The wheel's steering direction
     */
    abstract public double getRawDirection ();
    
    /**
     * Sets the target steering direction of the wheel, using the same rotational
     * measurement as {@link #getRawDirection()}. This means that if you were to pass 1.2 revolutions
     * as an argument, the input <b>should not wrap to the equivalent angle of 0.2 revolutions</b>,
     * and the module should instead perform whatever steering action is necessary in order to make the
     * output of {@code getDirection()} closer to 1.2.
     * @param targetDirection The target steering direction
     * @see #steerAndDrive(double, double)
     * @see #setDriveSpeed(double)
     */
    abstract public void setRawDirection (double targetDirection);
    
    /**
     * Sets the drive speed of the wheel on the interval [-1, 1].
     * @param speed The drive speed
     * @see #steerAndDrive(double, double)
     * @see #setRawDirection(double)
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
     * @see #setRawDirection(double)
     */
    public final void steerAndDrive (double targetDirection, double speed) {
        if (speed < 0 || speed > 1) throw new IllegalArgumentException("speed should be within range [-1, 1]");
        if (targetDirection >= 360 || targetDirection < 0) throw new IllegalArgumentException("targetDirection should be within range [0, 360)");
        
        // Finds current direction in degrees within range [0, 360)
        double currentDirection = getRawDirection() * 360;
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
        
        setRawDirection(moveDirection / 360 + getRawDirection());
        setDriveSpeed(speed * reverse);
    }
    
    /**
     * Gets the current steering direction, in degrees, on the interval [0, 360).
     * A direction of zero corresponds with directly forwards on the robot, and as
     * the direction increases, the steering direction is further clockwise from a
     * top-down view.
     * @return The steering direction of the wheel
     */
    public double getDirection () {
        double direction = getRawDirection() * 360;
        while (direction >= 360) direction -= 360;
        while (direction < 0) direction += 360;
        return direction;
    }
    
    /**
     * Checks whether or not the wheel's current steering direction is near a certain direction,
     * with a specified margin of error. Both {@code direction} and {@code marginOfError}
     * are measured in degrees. {@code direction} must be on the interval [0, 360), and
     * {@code marginOfError} must be on the interval (0, 360). A {@code direction} of zero
     * corresponds with directly forwards on the robot, and as {@code direction} increases,
     * the target steering direction moves clockwise from a top-down view.
     * @param direction     The steering direction to check against
     * @param marginOfError The acceptable margin of error, above or below {@code direction}
     * @return              Whether or not the current steering direction is within the
     * target range.
     * @see #checkWithin180Range(double, double)
     */
    public boolean checkWithinRange (double direction, double marginOfError) {
		double directionalDifference = direction - getDirection();
        while (directionalDifference > 180) directionalDifference -= 360;
        while (directionalDifference < -180) directionalDifference += 360;
        return directionalDifference >= -marginOfError && directionalDifference <= marginOfError;
	}
    
    /**
     * Does the same check as {@link #checkWithinRange(double, double)}, except it will also
     * return {@code true} if the wheel's current steering direction is within the range centered
     * at 180 degrees away from the target steering rotation.
     * @param direction     The steering direction to check against
     * @param marginOfError The acceptable margin of error, above or below {@code direction}
     * @return              Whether or not the current steering direction is within the
     * target range.
     */
    public boolean checkWithin180Range (double direction, double marginOfError) {
        double direction180 = direction + 180;
        if (direction180 >= 360) direction180 -= 360;
        return checkWithinRange(direction, marginOfError) || checkWithinRange(direction180, marginOfError);
    }
    
    /**
     * Stops all movement.
     */
    public void stop () {
        setDriveSpeed(0);
        stopSteering();
    }
    
}