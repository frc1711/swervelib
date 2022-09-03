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
     * Gets the number of inches traveled along the ground, according to the drive encoder.
	 * Driving the wheel in reverse should decrease this value, and driving forwards should
	 * increase it.
     * @return The inches this wheel has driven since the robot was enabled, with driving forwards
	 * increasing the measure and driving backwards decreasing it.
     */
    abstract public double getEncoderDistance ();
	
}