// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swerve.subsystems;

/**
 * Used by {@link swerve.drive.FESwerveDrive} to represent a module, with "full encoder" (FE)
 * functionality, meaning it has drive encoders in addition to the necessary rotational encoders.
 * <b>Note: Based on encoder functionality, the wheel should be facing directly forwards when
 * this subsystem is instantiated, or {@link #resetSteerEncoder()} should be used along with
 * a homing sequence.</b>
 * @author Gabriel Seaver
 */
abstract public class FESwerveWheel extends SwerveWheel {
    
    /**
     * Resets the drive encoder to a value of zero. This can be used to set a
     * reference point for determining the distance traveled over a period of time.
     * @see #getPositionDifference()
     */
    abstract public void resetDriveEncoder ();
    
    /**
     * Gets the drive encoder difference from the last drive encoder reset,
     * converted into inches traveled along the ground.
     * @return The inches traveled since the last {@link #resetDriveEncoder()} call
     */
    abstract public double getPositionDifference ();
    
}