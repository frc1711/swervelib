// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.util.odometry;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;
import frc.team1711.swerve.subsystems.AutoSwerveWheel;
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
    private Position position = new Position(Vector.ZERO, 0);
    
    /**
     * Creates a new {@link Odometry} object which tracks the position of an {@link AutoSwerveDrive} on the field.
     * This constructor method should not be called outside of the {@code AutoSwerveDrive} class. Use methods on the
     * {@code AutoSwerveDrive} class in order to access odometry functionality.
     * @param swerveDrive       The {@code AutoSwerveDrive} subsystem.
     * @param frontLeftWheel    The front left {@code AutoSwerveWheel}.
     * @param frontRightWheel   The front right {@code AutoSwerveWheel}.
     * @param rearLeftWheel     The rear left {@code AutoSwerveWheel}.
     * @param rearRightWheel    The rear right {@code AutoSwerveWheel}.
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
        
        position = position.addMovementVector(movement).withDirection(swerveDrive.getAbsoluteGyroAngle() - gyroYawOffset);
    }
    
    /**
     * Resets the robot's odometry to a given {@link Position}.
     * @param newPosition The new {@code Position} for the robot's odometry
     */
    public void resetPosition (Position newPosition) {
        // Sets the current robot's position to be the new position
        position = newPosition;
        
        // Sets the gyro yaw offset such that as we make new positions when the odometry is updated
        // the new position directions will match
        gyroYawOffset = swerveDrive.getAbsoluteGyroAngle() - position.getDirection();
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
    
}