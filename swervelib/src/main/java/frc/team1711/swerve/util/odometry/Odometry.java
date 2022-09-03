package frc.team1711.swerve.util.odometry;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;
import frc.team1711.swerve.subsystems.AutoSwerveWheel;
import frc.team1711.swerve.util.Vector;

/**
 * A class used in coordination with {@link AutoSwerveDrive} in order to track the position of the robot on the field.
 */
public class Odometry {
	
	private double
		frontLeftDistance = 0,
		frontRightDistance = 0,
		rearLeftDistance = 0,
		rearRightDistance = 0;
	
	private final AutoSwerveWheel
		frontLeftWheel,
		frontRightWheel,
		rearLeftWheel,
		rearRightWheel;
	
	private final AutoSwerveDrive swerveDrive;
	
	private Vector position = new Vector(0, 0);
	private double absoluteGyroAngle = 0;
	
	/**
	 * Creates a new {@code Odometry} object which tracks the position of an {@link AutoSwerveDrive} on the field.
	 * This constructor method should not be called outside of the {@code AutoSwerveDrive} class. Use methods on the
	 * {@code AutoSwerveDrive} class in order to access odometry functionality.
	 * @param swerveDrive
	 * @param frontLeftWheel
	 * @param frontRightWheel
	 * @param rearLeftWheel
	 * @param rearRightWheel
	 */
	public Odometry (
			AutoSwerveDrive swerveDrive,
			AutoSwerveWheel frontLeftWheel,
			AutoSwerveWheel frontRightWheel,
			AutoSwerveWheel rearLeftWheel,
			AutoSwerveWheel rearRightWheel) {
		this.swerveDrive = swerveDrive;
		this.frontLeftWheel = frontLeftWheel;
		this.frontRightWheel = frontRightWheel;
		this.rearLeftWheel = rearLeftWheel;
		this.rearRightWheel = rearRightWheel;
	}
	
	/**
	 * Used in order to update the estimated robot location on the field. This method is automatically called by
	 * {@link AutoSwerveDrive} every time the robot's movement kinematics are set.
	 */
	public void update () {
		final Vector frontLeftMovement = getWheelMovement(frontLeftWheel, frontLeftDistance);
		final Vector frontRightMovement = getWheelMovement(frontRightWheel, frontRightDistance);
		final Vector rearLeftMovement = getWheelMovement(rearLeftWheel, rearLeftDistance);
		final Vector rearRightMovement = getWheelMovement(rearRightWheel, rearRightDistance);
		
		final Vector movement = frontLeftMovement.add(frontRightMovement).add(rearLeftMovement).add(rearRightMovement).scale(0.25);
		position = position.add(movement);
		absoluteGyroAngle = swerveDrive.getAbsoluteGyroAngle();
	}
	
	private Vector getWheelMovement (AutoSwerveWheel wheel, double prevDistance) {
		final double direction = swerveDrive.getAbsoluteGyroAngle() + wheel.getDirection();
		final double magnitude = wheel.getEncoderDistance() - prevDistance;
		return Vector.fromPolarDegrees(direction, magnitude);
	}
	
}