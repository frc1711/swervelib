package frc.team1711.swerve.subsystems;

import frc.team1711.swerve.util.Angles;
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
     * @param widthToHeightRatio   The ratio from the track to the wheelbase (the distance between the centers
     * of the front or back wheels divided by the distance between the centers of the left or right wheels).
     */
    public GyroSwerveDrive (
        SwerveWheel flWheel,
        SwerveWheel frWheel,
        SwerveWheel rlWheel,
        SwerveWheel rrWheel,
        double widthToHeightRatio) {
        
        super(flWheel, frWheel, rlWheel, rrWheel, widthToHeightRatio);
    }
    
	/**
     * Drives the {@code SwerveDrive} given strafing and steering inputs,
     * all on the interval [-1, 1], where +y is forwards and +x is to the right.
	 * Strafing is field relative, not robot relative.
     * @param strafeX           The strafing speed in the x direction
     * @param strafeY           The strafing speed in the y direction
     * @param steering          The steering speed, where a positive value steers clockwise from a top-down point of view
     * @param useInputDeadbands Whether or not to treat {@code strafeX}, {@code strafeY}, and {@code steering} as UI
     * inputs (i.e. whether or not to apply the deadband set by {@link #setDeadband(double)} to these values). {@code true}
     * means the deadband will be applied.
     * @see #inputDrive(double, double, double, boolean)
	 * @see #steerAndDriveAll(double, double)
     */
    public void fieldRelativeInputDrive (double strafeX, double strafeY, double steering, boolean useInputDeadbands) {
        Vector strafeInput = new Vector(strafeX, strafeY);
		
		// strafeInput deadband
		if (useInputDeadbands) {
			strafeInput = accountForDeadband(strafeInput);
			steering = accountForDeadband(steering);
		}
        
        // Turns the strafeInput vector into a new vector with same magnitude but rotation adjusted for field relative
        final Vector fieldStrafeInput = strafeInput.toRotationDegrees(fieldRelToRobotRel(strafeInput.getRotationDegrees()));
        
        super.inputDrive(
			fieldStrafeInput.getX(),
			fieldStrafeInput.getY(),
			steering,
			false);
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
     * accurate {@link #fieldRelativeInputDrive(double, double, double, boolean)}.
     */
    public abstract void resetGyro ();
    
    private double fieldRelToRobotRel (double rotation) {
        return Angles.wrapDegrees(rotation - getGyroAngle());
    }
    
}