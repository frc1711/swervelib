// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;
import frc.team1711.swerve.util.Angles;
import frc.team1711.swerve.util.Vector;

/**
 * Drives the robot in a given direction without first turning the wheels. The wheels
 * will automatically turn while driving, but because they don't necessarily start off
 * being steered in the correct direction, driving will be slightly inaccurate.
 * @author Gabriel Seaver
 */
class AutonDriveSimple extends CommandBase {
    
    private final AutoSwerveDrive swerveDrive;
    
    private boolean finished;
    
    private final double
            correctionScalar,
            direction,
            distance,
            speed;

    private double initialGyroAngle;
    
    /**
     * Constructs an {@code AutonDriveSimple} command.
     * @param swerveDrive       The {@link AutoSwerveDrive} drive train
     * @param direction         The direction, in degrees, to travel in. Zero degrees corresponds with
     * directly forward relative to the robot, and an increase in {@code direction} corresponds with
     * a direction further clockwise from a top-down view.
     * @param distance          The distance to travel in the specified direction, in inches. This value
     * must be on the interval (0, infinity).
     * @param speed             The speed to travel at. This value must be on the interval (0, 1].
     * @param correctionScalar  The speed to turn at per degree offset from the intended direction. For
     * example, if the robot is going ten degrees to the right of where it wants to, it will make a
     * correction turn to the left at a speed of {@code 10*correctionScalar}. A recommended starting value
     * is {@code 0.01}.
     */
    AutonDriveSimple (AutoSwerveDrive swerveDrive, double direction, double distance, double speed, double correctionScalar) {
        this.correctionScalar = correctionScalar;
        this.swerveDrive = swerveDrive;
        this.direction = direction;
        this.distance = distance;
        this.speed = speed;
        
        finished = false;
        
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize () {
        swerveDrive.stop();
        swerveDrive.setDistanceReference();
        initialGyroAngle = swerveDrive.getGyroAngle();
    }
    
    @Override
    public void execute () {
        if (swerveDrive.getDistanceTraveled() < distance) {
            final Vector driveVector = Vector.fromPolarDegrees(direction, speed);
            swerveDrive.inputDrive(driveVector.getX(), driveVector.getY(), getCorrectionTurn(), false);
        } else {
            finished = true;
        }
    }
    
    private double getCorrectionTurn () {
        double correctionTurn = Angles.wrapDegreesZeroCenter(initialGyroAngle - swerveDrive.getGyroAngle());
        return correctionTurn * correctionScalar;
    }
    
    @Override
    public void end (boolean interrupted) {
        swerveDrive.stop();
    }
    
    @Override
    public boolean isFinished () {
        return finished;
    }
    
}