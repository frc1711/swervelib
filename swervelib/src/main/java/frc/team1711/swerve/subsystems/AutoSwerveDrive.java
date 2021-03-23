// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.subsystems;

import frc.team1711.swerve.util.Vector;

/**
 * Expands on the {@link SwerveDrive} for autonomous control. Uses swerve wheels with
 * drive encoders, in the form of {@link AutoSwerveWheel}. Includes gyros.
 * @author Gabriel Seaver
 */
public abstract class AutoSwerveDrive extends SwerveDrive {
    
    /**
     * Creates a new {@code AutoSwerveDrive} given {@link AutoSwerveWheel} wheels.
     * <b>Note: {@link #AutoSwerveDrive(AutoSwerveWheel, AutoSwerveWheel, AutoSwerveWheel, AutoSwerveWheel, double)}
     * should be used instead if the wheelbase and track are not equal.</b>
     * @param flWheel              The front left {@code SwerveWheel}
     * @param frWheel              The front right {@code SwerveWheel}
     * @param rlWheel              The rear left {@code SwerveWheel}
     * @param rrWheel              The rear right {@code SwerveWheel}
     */
    public AutoSwerveDrive (
        AutoSwerveWheel flWheel,
        AutoSwerveWheel frWheel,
        AutoSwerveWheel rlWheel,
        AutoSwerveWheel rrWheel) {
        
        this(flWheel, frWheel, rlWheel, rrWheel, 1);
    }
    
    /**
     * Creates a new {@code AutoSwerveDrive} given {@link AutoSwerveWheel} wheels.
     * @param flWheel              The front left {@code SwerveWheel}
     * @param frWheel              The front right {@code SwerveWheel}
     * @param rlWheel              The rear left {@code SwerveWheel}
     * @param rrWheel              The rear right {@code SwerveWheel}
     * @param widthToHeightRatio   The ratio from the track to the wheelbase (the distance between the centers
     * of the front or back wheels divided by the distance between the centers of the left or right wheels).
     * {@link #AutoSwerveDrive(AutoSwerveWheel, AutoSwerveWheel, AutoSwerveWheel, AutoSwerveWheel)} is recommended if this ratio is 1:1.
     */
    public AutoSwerveDrive (
        AutoSwerveWheel flWheel,
        AutoSwerveWheel frWheel,
        AutoSwerveWheel rlWheel,
        AutoSwerveWheel rrWheel,
        double widthToHeightRatio) {
        
        super(flWheel, frWheel, rlWheel, rrWheel, widthToHeightRatio);
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
    
    /**
     * Drives the {@code AutoSwerveDrive} given strafing and steering inputs,
     * all on the interval [-1, 1], where +y is forwards and +x is to the right.
     * Strafing is field relative, not robot relative.
     * @param strafeX   The strafe x input
     * @param strafeY   The strafe y input
     * @param steer     The steering input
     */
    public void fieldRelativeInputDrive (double strafeX, double strafeY, double steer) {
        final Vector strafeInput = new Vector(strafeX, strafeY);
        final Vector fieldStrafeInput = strafeInput.toRotationDegrees(fieldRelativeToRobotRelative(strafeInput.getRotationDegrees()));
        
        super.inputDrive(
                fieldStrafeInput.getX(),
                fieldStrafeInput.getY(),
                steer);
    }
    
    /**
     * Gets the gyro yaw angle on the range [0, 360) degrees.
     * @return The gyro yaw angle.
     */
    public abstract double getGyroAngle ();
    
    /**
     * Resets the gyro to a yaw angle of 0. It is recommended that this
     * be called when this {@code AutoSwerveDrive} object is first
     * instantiated, so the robot should be facing in the (field relative)
     * forward direction when this class is instantiated in order to have an
     * accurate {@link #fieldRelativeInputDrive(double, double, double)}.
     */
    public abstract void resetGyro ();
    
    private double fieldRelativeToRobotRelative (double rotation) {
        double moveRotation = rotation - getGyroAngle();
        while (moveRotation < 0) moveRotation += 360;
        while (moveRotation >= 360) moveRotation -= 360;
        return moveRotation;
    }
    
}