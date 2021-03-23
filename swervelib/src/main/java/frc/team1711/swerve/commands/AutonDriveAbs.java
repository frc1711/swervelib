// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.commands;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;

/**
 * Turns the entire {@link AutoSwerveDrive} body in a certain direction,
 * relative to the gyro's initialization orientation.
 * @author Gabriel Seaver
 */
public class AutonDriveAbs extends AutonDriveRel {
    
    /**
     * Constructs an {@code AutonDriveAbs} command.
     * @param swerveDrive   The {@link AutoSwerveDrive} drive train
     * @param direction     The direction, in degrees, to travel in. Zero degrees corresponds with
     * directly forward relative to the robot, and an increase in {@code direction} corresponds with
     * a direction further clockwise from a top-down view. This value must be on the interval [0, 360).
     * @param distance      The distance to travel in the specified direction, in inches. This value
     * must be on the interval (0, infinity).
     * @param speed         The speed to travel at. This value must be on the interval (0, 1].
     */
    public AutonDriveAbs (AutoSwerveDrive swerveDrive, double direction, double distance, double speed) {
        super(swerveDrive, direction - swerveDrive.getGyroAngle(), distance, speed);
    }
    
}