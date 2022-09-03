// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.subsystems;

import edu.wpi.first.wpilibj.interfaces.Gyro;

import frc.team1711.swerve.util.odometry.Odometry;
import frc.team1711.swerve.util.odometry.Odometry.Position;

/**
 * Expands on the {@link GyroSwerveDrive} for autonomous control, requiring
 * the use of {@link AutoSwerveWheel} rather than {@link SwerveWheel}. Uses
 * {@link Odometry} to track the position of the robot on the field.
 * @author Gabriel Seaver
 */
public abstract class AutoSwerveDrive extends GyroSwerveDrive {
    
    private final Odometry odometry;
    
    /**
     * Creates a new {@code AutoSwerveDrive} given {@link AutoSwerveWheel} wheels.
     * @param gyro                  The {@link Gyro} to be used for field-relative control
     * @param flWheel               The front left {@code AutoSwerveWheel}
     * @param frWheel               The front right {@code AutoSwerveWheel}
     * @param rlWheel               The rear left {@code AutoSwerveWheel}
     * @param rrWheel               The rear right {@code AutoSwerveWheel}
     * @param wheelbaseToTrackRatio	The distance between the centers of the left and right wheels divided
     * by the distance between the centers of the front and back wheels
     */
    public AutoSwerveDrive (
            Gyro gyro,
            AutoSwerveWheel flWheel,
            AutoSwerveWheel frWheel,
            AutoSwerveWheel rlWheel,
            AutoSwerveWheel rrWheel,
            double wheelbaseToTrackRatio) {
        
        super(gyro, flWheel, frWheel, rlWheel, rrWheel, wheelbaseToTrackRatio);
        
        odometry = new Odometry(this, flWheel, frWheel, rlWheel, rrWheel);
    }
    
    /**
     * Resets the robot's odometry to a given {@link Position}.
     * @param newPosition The new {@code Position} for the robot's odometry
     */
    public void resetPosition (Position newPosition) {
        odometry.resetPosition(newPosition);
    }
    
    /**
     * Gets the current {@link Position} of the robot on the field.
     * @return The robot's {@code Position}
     */
    public Position getPosition () {
        return odometry.getPosition();
    }
    
    @Override
    protected void updateOdometry () {
        odometry.update();
    }
    
}