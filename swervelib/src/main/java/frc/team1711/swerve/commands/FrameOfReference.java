// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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