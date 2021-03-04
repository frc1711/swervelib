// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swerve.drive;

import swerve.subsystems.FESwerveWheel;

/**
 * Expands on the {@link SwerveDrive} for better autonomous control. Uses swerve wheels with
 * drive encoders, in the form of {@link FESwerveWheel}.
 * @author Gabriel Seaver
 */
public class FESwerveDrive extends SwerveDrive {
    
    /**
     * Creates a new {@code FESwerveDrive} given {@link FESwerveWheel} wheels.
     * <b>Note: {@link #FESwerveDrive(FESwerveWheel, FESwerveWheel, FESwerveWheel, FESwerveWheel, double)}
     * should be used instead if the wheelbase and track are not equal.</b>
     * @param _flWheel              The front left {@code SwerveWheel}
     * @param _frWheel              The front right {@code SwerveWheel}
     * @param _rlWheel              The rear left {@code SwerveWheel}
     * @param _rrWheel              The rear right {@code SwerveWheel}
     */
    public FESwerveDrive (
        FESwerveWheel _flWheel,
        FESwerveWheel _frWheel,
        FESwerveWheel _rlWheel,
        FESwerveWheel _rrWheel) {
        
        super(_flWheel, _frWheel, _rlWheel, _rrWheel, 1);
    }
    
    /**
     * Creates a new {@code FESwerveDrive} given {@link FESwerveWheel} wheels.
     * @param _flWheel              The front left {@code SwerveWheel}
     * @param _frWheel              The front right {@code SwerveWheel}
     * @param _rlWheel              The rear left {@code SwerveWheel}
     * @param _rrWheel              The rear right {@code SwerveWheel}
     * @param _widthToHeightRatio   The ratio from the track to the wheelbase (the distance between the centers
     * of the front or back wheels divided by the distance between the centers of the left or right wheels).
     * {@link #FESwerveDrive(FESwerveWheel, FESwerveWheel, FESwerveWheel, FESwerveWheel)} is recommended if this ratio is 1:1.
     */
    public FESwerveDrive (
        FESwerveWheel _flWheel,
        FESwerveWheel _frWheel,
        FESwerveWheel _rlWheel,
        FESwerveWheel _rrWheel,
        double _widthToHeightRatio) {
        
        super(_flWheel, _frWheel, _rlWheel, _rrWheel, _widthToHeightRatio);
    }
    
    /**
     * Sets a distance reference on the encoders, such that the output of
     * {@link #getDistanceTraveled()} will be based on the distance from this reference.
     */
    public void setDistanceReference () {
        ((FESwerveWheel)flWheel).resetDriveEncoder();
        ((FESwerveWheel)frWheel).resetDriveEncoder();
        ((FESwerveWheel)rlWheel).resetDriveEncoder();
        ((FESwerveWheel)rrWheel).resetDriveEncoder();
    }
    
    /**
     * Gets the distance traveled, in inches, since the last distance reference was set.
     * This value is determined by the average distance traveled for each {@link FESwerveWheel},
     * so the return value of this method is <b>only going to be accurate if all
     * {@code FESwerveWheel} wheels are steered in the same direction</b> (or an equivalent angle).
     * @return The number of inches traveled
     * @see #setDistanceReference()
     */
    public double getDistanceTraveled () {
        return (Math.abs(((FESwerveWheel)flWheel).getPositionDifference()) +
                Math.abs(((FESwerveWheel)frWheel).getPositionDifference()) +
                Math.abs(((FESwerveWheel)rlWheel).getPositionDifference()) +
                Math.abs(((FESwerveWheel)rrWheel).getPositionDifference())) / 4;
    }
    
    @Override
    public String getDescription () {
        return "FESwerveDrive";
    }
    
}