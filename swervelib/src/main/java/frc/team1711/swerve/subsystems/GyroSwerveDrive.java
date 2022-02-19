// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.team1711.swerve.util.Angles;
import frc.team1711.swerve.util.InputHandler;
import frc.team1711.swerve.util.Vector;

/**
 * Expands on the {@link SwerveDrive} for field relative control, using a gyro.
 * @author Gabriel Seaver
 */
public abstract class GyroSwerveDrive extends SwerveDrive {
    
    /**
     * Creates a new {@code GyroSwerveDrive} given {@link SwerveWheel} wheels.
     * @param flWheel              The front left {@code SwerveWheel}
     * @param frWheel              The front right {@code SwerveWheel}
     * @param rlWheel              The rear left {@code SwerveWheel}
     * @param rrWheel              The rear right {@code SwerveWheel}
     * @param wheelbaseToTrackRatio		The distance between the centers of the left and right wheels divided
	 * by the distance between the centers of the front and back wheels
	 * @param swerveDrivingSpeeds  The {@link SwerveDrivingSpeeds} configuration
     */
    public GyroSwerveDrive (
        SwerveWheel flWheel,
        SwerveWheel frWheel,
        SwerveWheel rlWheel,
        SwerveWheel rrWheel,
        double wheelbaseToTrackRatio,
		SwerveDrivingSpeeds swerveDrivingSpeeds) {
        
        super(flWheel, frWheel, rlWheel, rrWheel, wheelbaseToTrackRatio, swerveDrivingSpeeds);
    }
    
	/**
     * Drives the {@code SwerveDrive} given strafing and steering inputs, all on the interval [-1, 1],
	 * where +{@code strafeY} is forwards and +{@code strafeX} is to the right. Inputs are assumed to be from a user-controlled
	 * device, so {@link SwerveDrivingSpeeds} are applied, along with an {@link InputHandler}. Strafing is
	 * field relative, not robot relative.
     * @param strafeX           The strafing speed in the x direction
     * @param strafeY           The strafing speed in the y direction
     * @param steering          The steering speed, where a positive value steers clockwise from a top-down point of view
     * @param inputHandler		The {@code InputHandler} to be used for converting user inputs into usable outputs.
	 * @see #userInputDrive(double, double, double, InputHandler)
     * @see #steerAndDriveAll(double, double)
     */
    public void fieldRelativeUserInputDrive (double strafeX, double strafeY, double steering, InputHandler inputHandler) {
        Vector strafeInput = new Vector(strafeX, strafeY);
		
		// strafe and steering inputs processing
		strafeInput = inputHandler.apply(strafeInput);
		steering = inputHandler.apply(steering);
        
        // Turns the strafeInput vector into a new vector with same magnitude but rotation adjusted for field relative
        final Vector fieldStrafeInput = strafeInput.toRotationDegrees(fieldRelToRobotRel(strafeInput.getRotationDegrees()));
        
        super.autoDrive(
			fieldStrafeInput.getX() * swerveDrivingSpeeds.strafeSpeed,
			fieldStrafeInput.getY() * swerveDrivingSpeeds.strafeSpeed,
			steering * swerveDrivingSpeeds.steerSpeed);
    }
	
	@Override
	public void initSendable (SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty("Gyro Angle", () -> getGyroAngle(), (x) -> {});
	}
    
    /**
     * Gets the gyro yaw angle on the range [0, 360) degrees.
     * @return The gyro yaw angle.
     */
    public abstract double getGyroAngle ();
    
    /**
     * Resets the gyro to a yaw angle of 0. It is recommended that this
     * be called when this {@code GyroSwerveDrive} object is first
     * instantiated, so the robot should be facing in the (field relative)
     * forward direction when this class is instantiated in order to have an
     * accurate {@link #fieldRelativeUserInputDrive(double, double, double, InputHandler)}.
     */
    public abstract void resetGyro ();
    
    private double fieldRelToRobotRel (double rotation) {
        return Angles.wrapDegrees(rotation - getGyroAngle());
    }
    
}