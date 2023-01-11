// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.util.odometry;

import frc.team1711.swerve.util.Angles;
import frc.team1711.swerve.util.Vector;

/**
 * Represents position of the robot on the field, with location and direction components. Used with the
 * {@link frc.team1711.swerve.subsystems.AutoSwerveDrive} subsystem.
 * @author Gabriel Seaver
 */
public class Position {
    
    private final Vector location;
    private final double direction;
    
    /**
     * Creates a new robot {@code Position}.
     * 
     * <p><b>The {@code direction} of the robot position is not the same measure as
     * {@link frc.team1711.swerve.subsystems.GyroSwerveDrive#getGyroAngle()}.
     * {@code getGyroAngle()} is only used for {@code GyroSwerveDrive.fieldRelativeUserInputDrive()}
     * (that is, field relative teleop), whereas {@code Position.direction} is used for autonomous functionality.</b></p>
     * 
     * @param location A {@code Vector} representing the robot's location on the field, measured in inches.
     * @param direction A {@code double} representing the direction angle of the robot, measured in degrees.
     */
    public Position (Vector location, double direction) {
        this.location = location;
        this.direction = Angles.wrapDegrees(direction);
    }
    
    /**
     * Adds a movement {@link Vector} to this {@link Position} to return a new {@code Position}.
     * @param movement The movement vector to add, measured in inches.
     * @return This same {@code Position}, with a movement {@code Vector} added to the location.
     */
    public Position addMovementVector (Vector movement) {
        return new Position(location.add(movement), direction);
    }
    
    /**
     * Returns a new {@link Position} with the same location but a different direction.
     * @param newDir The direction for the new position, measured in degrees.
     * @return The new {@code Position}.
     */
    public Position withDirection (double newDir) {
        return new Position(location, newDir);
    }
    
    /**
     * Gets the location of the robot on the field as a vector, measured in inches.
     * @return The location of the robot on the field.
     */
    public Vector getLocation () {
        return location;
    }
    
    /**
     * Gets the robot's direction relative to the field, in degrees. A turn to the right will increase this measure.
     * 
     * <p><b>The {@code direction} of the robot position is not the same measure as
     * {@link frc.team1711.swerve.subsystems.GyroSwerveDrive#getGyroAngle()}.
     * {@code getGyroAngle()} is only used for {@code GyroSwerveDrive.fieldRelativeUserInputDrive()}
     * (that is, field relative teleop), whereas {@code Position.direction} is used for autonomous functionality.</b></p>
     * 
     * @return The robot's direction
     */
    public double getDirection () {
        return direction;
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