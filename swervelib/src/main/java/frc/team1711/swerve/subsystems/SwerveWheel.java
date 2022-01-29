// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.subsystems;

import frc.team1711.swerve.util.Angles;

/**
 * An abstract class used by {@link SwerveDrive} to represent a
 * module. Each {@code SwerveWheel} contains a wheel which can steer in any
 * direction and drive forwards or backwards.
 * @author Gabriel Seaver
 */
abstract public class SwerveWheel {
    
    /**
     * Sets the drive speed of the wheel on the interval [-1, 1].
     * @param speed The drive speed
     * @see #steerAndDrive(double, double)
     */
    abstract protected void setDriveSpeed (double speed);
    
    /**
     * Immediately stops all steering movement.
     */
    abstract protected void stopSteering ();
    
    /**
     * Sets the drive speed of the wheel, along with the target steering direction
     * of the module. The drive speed of the wheel should be on the interval [0, 1].
     * A target steering direction of 0 degrees should correspond with moving directly forward,
     * where an increase in target steering direction corresponds with a clockwise movement
     * in target steering direction (from a top-down point of view).
     * @param targetDirection   The target steering direction, as specified above
     * @param speed             The drive speed of the wheel
     * @see #setDriveSpeed(double)
     * @see #setDirection(double)
     */
    protected final void steerAndDrive (double targetDirection, double speed) {
        if (speed < 0 || speed > 1) throw new IllegalArgumentException("speed should be within range [0, 1]");

        // Finds the number of degrees we need to turn and places on interval [-180, 180)
        double moveDirection = Angles.wrapDegreesZeroCenter(targetDirection - getDirection());
        
        // If the number of degrees we need to turn is closer
        // to 180 than 0, we turn the opposite way and go in reverse
        int reverse = 1;
        if (Math.abs(moveDirection) > 90) {
            reverse = -1;
            moveDirection += 180;
        }
        final double setDir = Angles.wrapDegrees(moveDirection + getDirection());

        // Sets drive speed and direction
        setDirection(setDir);
        setDriveSpeed(speed * reverse);
    }
    
    /**
     * Gets the current steering direction, in degrees, on the interval [0, 360).
     * A direction of zero corresponds with directly forwards on the robot, and as
     * the direction increases, the steering direction is further clockwise from a
     * top-down view.
     * @return The steering direction of the wheel
     * @see #setDirection(double)
     */
    abstract protected double getDirection ();
    
    /**
     * Sets the target steering direction of the wheel on the interval [0, 360), where zero degrees
     * is directly forwards and an increase in direction indicates a further clockwise target direction.
     * @param targetDirection The target steering direction
     * @see #getDirection()
     * @see #steerAndDrive(double, double)
     */
    abstract protected void setDirection (double targetDirection);
    
    /**
     * Checks whether or not the wheel's current steering direction is near a certain direction,
     * with a specified margin of error. Both {@code direction} and {@code marginOfError}
     * are measured in degrees. {@code marginOfError} must be on the interval (0, 360). A
     * {@code direction} of zero corresponds with directly forwards on the robot, and as
     * {@code direction} increases, the target steering direction moves clockwise from a
     * top-down view.
     * @param direction     The steering direction to check against
     * @param marginOfError The acceptable margin of error, above or below {@code direction}
     * @return              Whether or not the current steering direction is within the
     * target range.
     * @see #checkWithin180Range(double, double)
     */
    protected boolean checkWithinRange (double direction, double marginOfError) {
        // Gets absolute difference in directions
		double directionalDifference = Math.abs(Angles.wrapDegreesZeroCenter(direction - getDirection()));
        return directionalDifference <= marginOfError;
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
    protected boolean checkWithin180Range (double direction, double marginOfError) {
        double oppositeDirection = direction + 180; // Gets opposite direction
        return checkWithinRange(direction, marginOfError) || checkWithinRange(oppositeDirection, marginOfError);
    }
    
    /**
     * Stops all movement.
     */
    protected void stop () {
        setDriveSpeed(0);
        stopSteering();
    }
    
}