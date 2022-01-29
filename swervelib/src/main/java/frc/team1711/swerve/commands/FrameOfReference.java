// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.commands;

/**
 * The spatial orientation to be used as a frame of reference for an autonomous command.
 * There is robot-relative, {@link #ROBOT}, and field-relative, {@link #FIELD}.
 * @author Gabriel Seaver
 */
public enum FrameOfReference {
    /**
     * Denoting a frame of reference for an autonomous command that is relative
     * to the robot.
     */
    ROBOT,
    
    /**
     * Denoting a frame of reference for an autonomous command that is relative
     * to the field (or more precisely, relative to the gyro's initial orientation).
     */
    FIELD
}