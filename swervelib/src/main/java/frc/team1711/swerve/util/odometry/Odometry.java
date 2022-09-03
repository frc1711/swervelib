// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.util.odometry;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;
import frc.team1711.swerve.subsystems.AutoSwerveWheel;
import frc.team1711.swerve.util.Angles;
import frc.team1711.swerve.util.Vector;

/**
 * A class used in coordination with {@link AutoSwerveDrive} in order to track the position of the robot on the field.
 * @author Gabriel Seaver
 */
public class Odometry {
	
	private double
		frontLeftDistance,
		frontRightDistance,
		rearLeftDistance,
		rearRightDistance;
	
	private final AutoSwerveWheel
		frontLeftWheel,
		frontRightWheel,
		rearLeftWheel,
		rearRightWheel;
	
	private final AutoSwerveDrive swerveDrive;
	
	private double gyroYawOffset;
	private Position position = new Position(new Vector(0, 0), 0);
	
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
		
		gyroYawOffset = swerveDrive.getAbsoluteGyroAngle();
	}
	
	/**
	 * Used in order to update the estimated robot location on the field. This method is automatically called by
	 * {@link AutoSwerveDrive} every time the robot's movement kinematics are set.
	 */
	public void update () {
		final Vector frontLeftMovement = getWheelMovement(frontLeftWheel, frontLeftDistance),
			frontRightMovement = getWheelMovement(frontRightWheel, frontRightDistance),
			rearLeftMovement = getWheelMovement(rearLeftWheel, rearLeftDistance),
			rearRightMovement = getWheelMovement(rearRightWheel, rearRightDistance),
			movement = frontLeftMovement.add(frontRightMovement).add(rearLeftMovement).add(rearRightMovement).scale(0.25);
		
		position = position.getPositionFromMovement(movement, swerveDrive.getAbsoluteGyroAngle() - gyroYawOffset);
	}
	
	/**
	 * Resets the robot's odometry to a given {@link Position}.
	 * @param newPosition The new {@code Position} for the robot's odometry
	 */
	public void resetPosition (Position newPosition) {
		// Sets the current robot's position to be the new position
		position = newPosition;
		
		// Sets the gyro yaw offset such that as we make new positions when the odometry is updated
		// the new position yaws will match
		gyroYawOffset = swerveDrive.getAbsoluteGyroAngle() - position.getYaw();
	}
	
	/**
	 * Gets the current {@link Position} of the robot on the field.
	 * @return The robot's {@code Position}
	 */
	public Position getPosition () {
		update(); // Just in case there it's been almost one roborio cycle since the last time the odometry was updated
		return position;
	}
	
	private Vector getWheelMovement (AutoSwerveWheel wheel, double prevDistance) {
		final double direction = swerveDrive.getAbsoluteGyroAngle() + wheel.getDirection();
		final double magnitude = wheel.getEncoderDistance() - prevDistance;
		return Vector.fromPolarDegrees(direction, magnitude);
	}
	
	/**
	 * Represents position of the robot on the field, with location and yaw components. Used with the
	 * {@link AutoSwerveDrive} subsystem.
	 */
	public static class Position {
		
		private final Vector location;
		private final double yaw;
		
		public Position (Vector location, double yaw) {
			this.location = location;
			this.yaw = yaw;
		}
		
		private Position getPositionFromMovement (Vector movement, double newYaw) {
			return new Position(location.add(movement), Angles.wrapDegrees(newYaw));
		}
		
		/**
		 * Gets the location of the robot on the field as a vector, with units measured in inches.
		 * @return The location of the robot on the field
		 */
		public Vector getLocation () {
			return location;
		}
		
		/**
		 * Gets the robot's yaw on the field, in degrees. A turn to the right represents an increase in yaw.
		 * @return The robot's yaw angle
		 */
		public double getYaw () {
			return yaw;
		}
		
		/**
		 * Gets a vector representing the movement required to move the robot from this position to
		 * another on the field.
		 * @param other The position to get the movement to
		 * @return A vector representing the movement from this position to another. Units are in inches.
		 */
		public Vector movementTo (Position other) {
			return other.location.subtract(this.location);
		}
		
		/**
		 * Gets the distance from this position to another on the field, in inches.
		 * @param other The position to get the distance from
		 * @return The distance from this position to the other
		 */
		public double distanceFrom (Position other) {
			return movementTo(other).getMagnitude();
		}
		
	}
	
}