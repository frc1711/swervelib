// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.subsystems;

/**
 * Expands on the {@link SwerveDrive} for autonomous control. Uses swerve wheels with
 * drive encoders, in the form of {@link AutoSwerveWheel}.
 * @author Gabriel Seaver
 */
public class AutoSwerveDrive extends SwerveDrive {
    
    /**
     * Creates a new {@code AutoSwerveDrive} given {@link AutoSwerveWheel} wheels.
     * <b>Note: {@link #AutoSwerveDrive(AutoSwerveWheel, AutoSwerveWheel, AutoSwerveWheel, AutoSwerveWheel, double)}
     * should be used instead if the wheelbase and track are not equal.</b>
     * @param _flWheel              The front left {@code SwerveWheel}
     * @param _frWheel              The front right {@code SwerveWheel}
     * @param _rlWheel              The rear left {@code SwerveWheel}
     * @param _rrWheel              The rear right {@code SwerveWheel}
     */
    public AutoSwerveDrive (
        AutoSwerveWheel _flWheel,
        AutoSwerveWheel _frWheel,
        AutoSwerveWheel _rlWheel,
        AutoSwerveWheel _rrWheel) {
        
        super(_flWheel, _frWheel, _rlWheel, _rrWheel, 1);
    }
    
    /**
     * Creates a new {@code AutoSwerveDrive} given {@link AutoSwerveWheel} wheels.
     * @param _flWheel              The front left {@code SwerveWheel}
     * @param _frWheel              The front right {@code SwerveWheel}
     * @param _rlWheel              The rear left {@code SwerveWheel}
     * @param _rrWheel              The rear right {@code SwerveWheel}
     * @param _widthToHeightRatio   The ratio from the track to the wheelbase (the distance between the centers
     * of the front or back wheels divided by the distance between the centers of the left or right wheels).
     * {@link #AutoSwerveDrive(AutoSwerveWheel, AutoSwerveWheel, AutoSwerveWheel, AutoSwerveWheel)} is recommended if this ratio is 1:1.
     */
    public AutoSwerveDrive (
        AutoSwerveWheel _flWheel,
        AutoSwerveWheel _frWheel,
        AutoSwerveWheel _rlWheel,
        AutoSwerveWheel _rrWheel,
        double _widthToHeightRatio) {
        
        super(_flWheel, _frWheel, _rlWheel, _rrWheel, _widthToHeightRatio);
    }
    
    /**
     * Sets a distance reference on the encoders, such that the output of
     * {@link #getDistanceTraveled()} will be based on the distance from this reference.
     */
    public void setDistanceReference () {
        ((AutoSwerveWheel)flWheel).resetDriveEncoder();
        ((AutoSwerveWheel)frWheel).resetDriveEncoder();
        ((AutoSwerveWheel)rlWheel).resetDriveEncoder();
        ((AutoSwerveWheel)rrWheel).resetDriveEncoder();
    }
    
    /**
     * Gets the distance traveled, in inches, since the last distance reference was set.
     * This value is determined by the average distance traveled for each {@link AutoSwerveWheel},
     * so the return value of this method is <b>only going to be accurate if all
     * {@code AutoSwerveWheel} wheels are steered in the same direction</b> (or an equivalent angle).
     * @return The number of inches traveled
     * @see #setDistanceReference()
     */
    public double getDistanceTraveled () {
        return (Math.abs(((AutoSwerveWheel)flWheel).getPositionDifference()) +
                Math.abs(((AutoSwerveWheel)frWheel).getPositionDifference()) +
                Math.abs(((AutoSwerveWheel)rlWheel).getPositionDifference()) +
                Math.abs(((AutoSwerveWheel)rrWheel).getPositionDifference())) / 4;
    }
    
}