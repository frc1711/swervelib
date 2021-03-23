// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team1711.swerve.subsystems.AutoSwerveDrive;

/**
 * A command which drives a given {@link frc.team1711.swerve.subsystems.AutoSwerveDrive} in any
 * direction, without turning, relative to the robot.
 * @author Gabriel Seaver
 */
public class AutonDrive extends SequentialCommandGroup {
    
    /**
     * Constructs an {@code AutonDrive} command.
     * @param swerveDrive       The {@link AutoSwerveDrive} drive train
     * @param direction         The direction, in degrees, to travel in. Zero degrees corresponds with
     * directly forward, and an increase in {@code direction} corresponds with
     * a direction further clockwise from a top-down view. This value must be on the interval [0, 360).
     * @param distance          The distance to travel in the specified direction, in inches. This value
     * must be on the interval (0, infinity).
     * @param speed             The speed to travel at. This value must be on the interval (0, 1].
     * @param frameOfReference  The {@link FrameOfReference} for this autonomous command.
     */
    public AutonDrive (AutoSwerveDrive swerveDrive, double direction, double distance, double speed, FrameOfReference frameOfReference) {
        if (frameOfReference == FrameOfReference.FIELD) direction -= swerveDrive.getGyroAngle();
        while (direction >= 180) direction -= 360;
        while (direction < -180) direction += 360;
        addCommands(
            new AutonWheelTurn(swerveDrive, direction),
            new AutonDriveSimple(swerveDrive, direction, distance, speed));
    }
    
}