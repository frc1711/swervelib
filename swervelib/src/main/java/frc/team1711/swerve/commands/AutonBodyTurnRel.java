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
public class AutonBodyTurnRel extends AutonBodyTurnAbs {
    
    /**
     * Constructs a new {@code AutonBodyTurnRel}.
     * @param swerveDrive   The {@link AutoSwerveDrive} drive train
     * @param direction     The direction, in degrees, to turn the body towards. Zero degrees
     * corresponds with directly forward relative to the robot, and an increase in {@code direction}
     * corresponds with a direction further clockwise from a top-down view. This value must be on
     * the interval [0, 360).
     */
    public AutonBodyTurnRel (AutoSwerveDrive swerveDrive, double direction) {
        super(swerveDrive, swerveDrive.getGyroAngle() + direction);
    }
    
}