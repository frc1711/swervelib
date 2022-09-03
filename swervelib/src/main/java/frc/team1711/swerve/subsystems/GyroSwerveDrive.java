// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.subsystems;

import edu.wpi.first.wpilibj.interfaces.Gyro;

import frc.team1711.swerve.util.Angles;
import frc.team1711.swerve.util.Vector;

/**
 * Expands on the {@link SwerveDrive} for field relative control, using a gyro.
 * @author Gabriel Seaver
 */
public abstract class GyroSwerveDrive extends SwerveDrive {
    
	private final Gyro gyro;
	private double gyroResetAngle;
	
    /**
     * Creates a new {@code GyroSwerveDrive} given {@link SwerveWheel} wheels.
	 * @param gyro					The {@link Gyro} to be used for field-relative control
     * @param flWheel				The front left {@code SwerveWheel}
     * @param frWheel				The front right {@code SwerveWheel}
     * @param rlWheel				The rear left {@code SwerveWheel}
     * @param rrWheel				The rear right {@code SwerveWheel}
     * @param wheelbaseToTrackRatio	The distance between the centers of the left and right wheels divided
	 * by the distance between the centers of the front and back wheels
     */
    public GyroSwerveDrive (
		Gyro gyro,
        SwerveWheel flWheel,
        SwerveWheel frWheel,
        SwerveWheel rlWheel,
        SwerveWheel rrWheel,
        double wheelbaseToTrackRatio) {
        
        super(flWheel, frWheel, rlWheel, rrWheel, wheelbaseToTrackRatio);
		this.gyro = gyro;
		gyroResetAngle = gyro.getAngle();
    }
    
	/**
     * Drives the {@code SwerveDrive} given strafing and steering inputs, all on the interval [-1, 1],
	 * where +{@code strafeY} is forwards and +{@code strafeX} is to the right. Inputs are assumed to be from a user-controlled
	 * device, so {@link ControlsConfig} is applied. Strafing is field relative, not robot relative.
     * @param strafeX           The strafing speed in the x direction
     * @param strafeY           The strafing speed in the y direction
     * @param steering          The steering speed, where a positive value steers clockwise from a top-down point of view
     * @param controlsConfig	The {@code ControlsConfig} to be used for relative driving speeds and processing of user inputs
	 * @see #userInputDrive(double, double, double, ControlsConfig)
     * @see #steerAndDriveAll(double, double)
     */
    public void fieldRelativeUserInputDrive (double strafeX, double strafeY, double steering, ControlsConfig controlsConfig) {
        Vector strafeInput = new Vector(strafeX, strafeY);
		
		// strafe and steering inputs processing
		strafeInput = controlsConfig.inputHandler.apply(strafeInput);
		steering = controlsConfig.inputHandler.apply(steering);
        
        // Turns the strafeInput vector into a new vector with same magnitude but rotation adjusted for field relative
        final Vector fieldStrafeInput = strafeInput.toRotationDegrees(fieldRelToRobotRel(strafeInput.getRotationDegrees()));
        
        autoDrive(
			fieldStrafeInput.getX() * controlsConfig.strafeSpeed,
			fieldStrafeInput.getY() * controlsConfig.strafeSpeed,
			steering * controlsConfig.steerSpeed);
    }
    
    /**
     * Gets the gyro yaw angle on the range [0, 360) degrees.
     * @return The gyro yaw angle.
	 * 
	 * @see #getAbsoluteGyroAngle()
	 * @see #resetGyro()
     */
    public double getGyroAngle () {
		return Angles.wrapDegrees(getAbsoluteGyroAngle() - gyroResetAngle);
	}
	
	/**
	 * Gets the absolute gyro angle, unaffected by {@link #resetGyro()}. The angle is not
	 * constricted to being between 0 and 360 degrees, and will continuously increase or
	 * decrease as the robot turns past full rotations.
	 * @return The gyro yaw angle.
	 * 
	 * @see #getGyroAngle()
	 */
	public double getAbsoluteGyroAngle () {
		return gyro.getAngle();
	}
    
    /**
     * Resets the gyro heading to {@code toAngle} for {@link #getGyroAngle()} but does not affect
	 * {@link #getAbsoluteGyroAngle()}.
	 * @param toAngle The angle to reset the gyro heading to.
	 * 
	 * @see #resetGyro()
     */
    public void resetGyro (double toAngle) {
		gyroResetAngle = getAbsoluteGyroAngle() - toAngle;
	}
	
	/**
	 * Resets the gyro heading to zero for {@link #getGyroAngle()} but does not affect
	 * {@link #getAbsoluteGyroAngle()}.
	 * 
	 * @see #resetGyro(double)
	 */
	public void resetGyro () {
		resetGyro(0);
	}
    
    private double fieldRelToRobotRel (double rotation) {
        return Angles.wrapDegrees(rotation - getGyroAngle());
    }
    
}