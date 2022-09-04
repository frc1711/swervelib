// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;
import frc.team1711.swerve.util.Vector;
import frc.team1711.swerve.util.odometry.Odometry.Position;

/**
 * Drives the robot in a given direction without first turning the wheels. The wheels
 * will automatically turn while driving, but because they don't necessarily start off
 * being steered in the correct direction, driving will be slightly inaccurate.
 * @author Gabriel Seaver
 */
class AutonDriveSimple extends CommandBase {
    
    private final AutoSwerveDrive swerveDrive;
    
    private boolean finished;
    
    // robotMovement and robotDirection can be used in the constructor method as
    // alternatives to finalPosition, but in the initialization method they'll be
    // converted to finalPosition anyway
    private RobotMovement robotMovement;
    private RobotDirection robotDirection;
    
    private Position finalPosition;
    private final double speed, turnCorrection, distMarginOfError;
    
    AutonDriveSimple (
            AutoSwerveDrive swerveDrive,
            RobotMovement robotMovement,
            RobotDirection robotDirection,
            double speed,
            double turnCorrection,
            double distMarginOfError) {
        // In the initialization method, robotMovement and robotDirection will be
        // converted into an equivalent finalPosition value
        this.robotMovement = robotMovement;
        this.robotDirection = robotDirection;
        
        this.swerveDrive = swerveDrive;
        this.speed = speed;
        this.turnCorrection = turnCorrection;
        this.distMarginOfError = distMarginOfError;
        
        finished = false;
        addRequirements(swerveDrive);
    }
    
    AutonDriveSimple (
            AutoSwerveDrive swerveDrive,
            Position finalPosition,
            double speed,
            double turnCorrection,
            double distMarginOfError) {
        this.finalPosition = finalPosition;
        
        this.swerveDrive = swerveDrive;
        this.speed = speed;
        this.turnCorrection = turnCorrection;
        this.distMarginOfError = distMarginOfError;
        
        finished = false;
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize () {
        swerveDrive.stop();
        
        // Convert movementVector and frameOfReference into finalPosition if necessary
        finalPosition = getFinalPosition();
    }
    
    public Position getFinalPosition () {
        return new Position(
            robotMovement.toFieldRel(swerveDrive.getPosition()),
            robotDirection.toFieldRel(swerveDrive.getPosition()));
    }
    
    @Override
    public void execute () {
        // remainingMovement is the vector representing the rest of the movement until the auton's endpoint (field relative)
        Vector remainingMovement = swerveDrive.getPosition().movementTo(finalPosition);
        
        if (remainingMovement.getMagnitude() <= distMarginOfError) {
            
            // If the remaining movement is close enough to the endpoint, just stop
            swerveDrive.stop();
            finished = true;
        } else {
            
            // Otherwise, the robot still needs to move. Here movement represents the movement for
            // this roboRIO cycle only
            Vector movement = remainingMovement.scale(speed / remainingMovement.getMagnitude());
            
            // Converts movement to be robot relative again
            movement = movement.toRotationDegrees(movement.getRotationDegrees() - swerveDrive.getPosition().getDirection());
            
            // Drives according to the x and y components of the movement vector
            swerveDrive.autoDrive(movement.getX(), movement.getY(), 0);
        }
    }
    
    @Override
    public void end (boolean interrupted) {
        swerveDrive.stop();
    }
    
    @Override
    public boolean isFinished () {
        return finished;
    }
    
    /**
     * Represents a movement of the robot as a {@link Vector} and a {@link FrameOfReference}. This
     * movement only includes strafing movement, not turning.
     */
    public static class RobotMovement {
        
        private final Vector movement;
        private final FrameOfReference frameOfReference;
        
        /**
         * Creates a new {@link RobotMovement} instance.
         * @param movement              A {@link Vector} which represents the movement of the robot. A positive y value
         * represents forward movement, and a positive x value represents movement to the right. Measured in inches.
         * @param frameOfReference      A {@link FrameOfReference} which indicates how the movement {@code Vector} is to
         * be interpreted. If {@code frameOfReference} is {@link FrameOfReference#FIELD}, then the movement {@code Vector}
         * will be relative to however the robot's {@link Position} on the field was last reset with
         * {@link AutoSwerveDrive#resetPosition(Position)}. If {@code frameOfReference} is
         * {@link FrameOfReference#ROBOT}, then the movement {@code Vector} will be relative to the robot itself.
         */
        public RobotMovement (Vector movement, FrameOfReference frameOfReference) {
            this.movement = movement;
            this.frameOfReference = frameOfReference;
        }
        
        /**
         * Converts the movement vector to be field relative (if it isn't already explicitly field relative)
         * given the robot's current {@link Position}.
         * @param position The robot's {@code Position} on the field
         * @return The equivalent field relative movement {@link Vector}, measured in inches
         */
        public Vector toFieldRel (Position position) {
            // If the frame of reference is already field-relative, return
            // the movement vector as-is
            if (frameOfReference == FrameOfReference.FIELD) return movement;
            
            // If the frame of reference is robot-relative, first get the new
            // direction the robot would have to travel in
            final double newDir = movement.getRotationDegrees() + position.getDirection();
            
            // Return the movement vector rotated in the correct direction
            return movement.toRotationDegrees(newDir);
        }
        
    }
    
    /**
     * Represents a direction or turn of the robot, containing a {@link FrameOfReference}.
     */
    public static class RobotDirection {
        
        private final double direction;
        private final FrameOfReference frameOfReference;
        
        /**
         * Creates a new {@link RobotDirection} instance.
         * @param direction             The {@code double} which represents the turn or final direction
         * of the robot. An angle of zero is facing forwards or upwards on the xy plane, and as the angle
         * increases it progresses clockwise. Measured in degrees.
         * @param frameOfReference      The {@link FrameOfReference} for the final direction. If
         * this is {@link FrameOfReference#ROBOT}, the given {@code direction} will be considered
         * a turn relative to the robot's current position. If this is {@link FrameOfReference#FIELD},
         * then the {@code direction} will be relative to the field, based on however the robot's
         * {@link Position} on the field was last reset with {@link AutoSwerveDrive#resetPosition(Position)}.
         */
        public RobotDirection (double direction, FrameOfReference frameOfReference) {
            this.direction = direction;
            this.frameOfReference = frameOfReference;
        }
        
        /**
         * Converts the direction to be field relative (if it isn't already explicitly field relative)
         * given the robot's current {@link Position}.
         * @param position The robot's {@code Position} on the field
         * @return The equivalent field relative direction measured in degrees
         */
        public double toFieldRel (Position position) {
            // If the frame of reference is already field-relative, do nothing
            if (frameOfReference == FrameOfReference.FIELD) return direction;
            
            // If the frame of reference is robot-relative, simply add the current robot direction
            return direction + position.getDirection();
        }
        
    }
    
}