// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.subsystems;

/**
 * Used by {@link AutoSwerveDrive} to represent a module with encoders on the drive motors
 * (in addition to the encoders on the steering motors which are necessary for basic swerve
 * functionality).
 * @author Gabriel Seaver
 */
abstract public class AutoSwerveWheel extends SwerveWheel {
    
    /**
     * Resets the drive encoder to a value of zero. This can be used to set a
     * reference point for determining the distance traveled over a period of time.
     * @see #getPositionDifference()
     */
    abstract protected void resetDriveEncoder ();
    
    /**
     * Gets the drive encoder difference from the last drive encoder reset,
     * converted into inches traveled along the ground.
     * @return The inches traveled since the last {@link #resetDriveEncoder()} call
     */
    abstract protected double getPositionDifference ();
    
}