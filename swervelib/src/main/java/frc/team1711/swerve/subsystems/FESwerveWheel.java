// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.subsystems;

/**
 * Used by {@link FESwerveDrive} to represent a module, with "full encoder" (FE)
 * functionality, meaning it has drive encoders in addition to the necessary rotational encoders.
 * @author Gabriel Seaver
 */
abstract public class FESwerveWheel extends SwerveWheel {
    
    /**
     * Resets the drive encoder to a value of zero. This can be used to set a
     * reference point for determining the distance traveled over a period of time.
     * @see #getPositionDifference()
     */
    abstract protected void resetDriveEncoder ();
    
    /**
     * Gets the drive encoder difference from the last drive encoder reset,
     * converted into inches traveled along the ground.
     * @return The inches traveled since the last {@link #resetDriveEncoder()} call
     */
    abstract protected double getPositionDifference ();
    
}