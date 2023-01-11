// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.util.odometry;

import frc.team1711.swerve.commands.FrameOfReference;
import frc.team1711.swerve.util.Vector;

/**
 * Represents an autonomous movement of the robot (only strafing, not turning). {@code RobotMovement} instances
 * include both information on the movement itself, and how the robot is to perform the movement.
 * @author Gabriel Seaver
 * 
 * @see RobotTurn
 * @see frc.team1711.swerve.commands.AutonDrive
 */
public class RobotMovement {
    
    /**
     * A {@code RobotMovement} describing no movement whatsoever.
     */
    public static final RobotMovement NONE = new RobotMovement(Vector.ZERO, FrameOfReference.ROBOT, MovementManner.NONE);
    
    private final Vector movement;
    private final FrameOfReference frameOfReference;
    private final MovementManner manner;
    
    /**
     * Creates a new {@link RobotMovement} instance.
     * @param movement              A {@link Vector} which represents the movement of the robot. A positive y value
     * represents forward movement, and a positive x value represents movement to the right. Measured in inches.
     * @param frameOfReference      A {@link FrameOfReference} which indicates how the movement {@code Vector} is to
     * be interpreted. If {@code frameOfReference} is {@link FrameOfReference#FIELD}, then the movement {@code Vector}
     * will be relative to however the robot's {@link Position} on the field was last reset with
     * {@link frc.team1711.swerve.subsystems.AutoSwerveDrive#resetPosition(Position)}. If {@code frameOfReference} is
     * {@link FrameOfReference#ROBOT}, then the movement {@code Vector} will be relative to the robot itself.
     * @param manner                A {@link MovementManner} which contains information on the way the movement
     * along the path is to be performed by the robot.
     */
    public RobotMovement (Vector movement, FrameOfReference frameOfReference, MovementManner manner) {
        this.movement = movement;
        this.frameOfReference = frameOfReference;
        this.manner = manner;
    }
    
    /**
     * Gets the {@link MovementManner} which describes how the robot should move along the given path.
     * @return The {@code MovementManner} associated with this path.
     */
    public MovementManner getManner () {
        return manner;
    }
    
    /**
     * Converts the movement vector to be field relative (if it isn't already explicitly field relative)
     * given the robot's current {@link Position}.
     * @param position The robot's {@code Position} on the field
     * @return The equivalent field relative movement {@link Vector}, measured in inches
     */
    public Vector toFieldRel (Position position) {
        // If the frame of reference is already field-relative, return
        // the movement vector as-is
        if (frameOfReference == FrameOfReference.FIELD) return movement;
        
        // If the frame of reference is robot-relative, first get the new
        // direction the robot would have to travel in
        final double newDir = movement.getRotationDegrees() + position.getDirection();
        
        // Return the movement vector rotated in the correct direction
        return movement.toRotationDegrees(newDir);
    }
    
}