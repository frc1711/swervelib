// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.subsystems;

/**
 * Expands on the {@link GyroSwerveDrive} for autonomous control, requiring
 * the use of {@link AutoSwerveWheel} rather than {@link SwerveWheel}.
 * @author Gabriel Seaver
 */
public abstract class AutoSwerveDrive extends GyroSwerveDrive {
	
	/**
     * Creates a new {@code AutoSwerveDrive} given {@link AutoSwerveWheel} wheels.
     * @param flWheel              The front left {@code AutoSwerveWheel}
     * @param frWheel              The front right {@code AutoSwerveWheel}
     * @param rlWheel              The rear left {@code AutoSwerveWheel}
     * @param rrWheel              The rear right {@code AutoSwerveWheel}
     * @param widthToHeightRatio   The ratio from the track to the wheelbase (the distance between the centers
     * of the front or back wheels divided by the distance between the centers of the left or right wheels).
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
	
}