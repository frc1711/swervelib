// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.util.odometry;

import frc.team1711.swerve.commands.FrameOfReference;
import frc.team1711.swerve.subsystems.AutoSwerveDrive;

/**
 * Represents an autonomous robot turn, including both the turn itself and the way the robot makes the turn.
 * @author Gabriel Seaver
 * 
 * @see RobotMovement
 */
public class RobotTurn {
    
    private final double direction;
    private final FrameOfReference frameOfReference;
    private final TurnManner manner;
    
    /**
     * Creates a new {@link RobotTurn} instance.
     * @param direction             The {@code double} which represents the turn or final direction
     * of the robot. An angle of zero is facing forwards or upwards on the xy plane, and as the angle
     * increases it progresses clockwise. Measured in degrees.
     * @param frameOfReference      The {@link FrameOfReference} for the final direction. If
     * this is {@link FrameOfReference#ROBOT}, the given {@code direction} will be considered
     * a turn relative to the robot's current position (i.e. a {@code direction} of {@code 30}
     * would represent a turn 30 degrees to the right from where the robot started the path).
     * If {@code frameOfReference} is {@link FrameOfReference#FIELD}, then the {@code direction}
     * will be relative to the field, based on however the robot's {@link Position} on the field
     * was last reset with {@link AutoSwerveDrive#resetPosition(Position)}.
     * @param manner                A {@link TurnManner} which contains information on the way the turn
     * is to be performed by the robot.
     */
    public RobotTurn (double direction, FrameOfReference frameOfReference, TurnManner manner) {
        this.direction = direction;
        this.frameOfReference = frameOfReference;
        this.manner = manner;
    }
    
    /**
     * Gets the {@link TurnManner} which describes how the robot should turn.
     * @return The {@code TurnManner} associated with this autonomous turn.
     */
    public TurnManner getManner () {
        return manner;
    }
    
    /**
     * Converts the direction to be field relative (if it isn't already explicitly field relative)
     * given the robot's current {@link Position}.
     * @param position The robot's {@code Position} on the field
     * @return The equivalent field relative direction measured in degrees
     */
    public double toFieldRel (Position position) {
        // If the frame of reference is already field-relative, do nothing
        if (frameOfReference == FrameOfReference.FIELD) return direction;
        
        // If the frame of reference is robot-relative, simply add the current robot direction
        return direction + position.getDirection();
    }
    
}