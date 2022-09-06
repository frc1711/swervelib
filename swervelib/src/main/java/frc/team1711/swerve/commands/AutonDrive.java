// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;
import frc.team1711.swerve.util.Angles;
import frc.team1711.swerve.util.Vector;
import frc.team1711.swerve.util.odometry.MovementManner;
import frc.team1711.swerve.util.odometry.Position;
import frc.team1711.swerve.util.odometry.RobotMovement;
import frc.team1711.swerve.util.odometry.RobotTurn;
import frc.team1711.swerve.util.odometry.TurnManner;

/**
 * A command to drive the robot to a given {@link Position} on the field, or to drive
 * using robot-relative or field-relative {@link RobotTurn} and {@link RobotMovement}
 * descriptor objects.
 * @author Gabriel Seaver
 */
public class AutonDrive extends CommandBase {
    
    private final AutoSwerveDrive swerveDrive;
    
    private boolean finished = false;
    private Position initialPosition;
    
    // The constructor provides one of the two following options.
    // Later on we convert them to fill in the gaps
    
    // CONSTRUCTOR OPTION #1:
    private RobotMovement robotMovement;
    private RobotTurn robotTurn;
    
    /**
     * Construct an {@link AutonDrive} command given an {@link AutoSwerveDrive} subsystem instance,
     * a {@link RobotMovement}, and a {@link RobotTurn}.
     * @param swerveDrive       A {@code AutoSwerveDrive} swerve drive subsystem.
     * @param robotMovement     A {@code RobotMovement} describing where and how to make the robot strafe
     * across the field. This can be robot relative or field relative.
     * @param robotTurn         A {@code RobotTurn} describing where and how to make the robot turn. This
     * can be to an exact field-relative direction or a turn relative to the robot.
     */
    public AutonDrive (
            AutoSwerveDrive swerveDrive,
            RobotMovement robotMovement,
            RobotTurn robotTurn) {
        this.robotMovement = robotMovement;
        this.robotTurn = robotTurn;
        
        this.swerveDrive = swerveDrive;
        
        addRequirements(swerveDrive);
    }
    
    // CONSTRUCTOR OPTION #2:
    private Position finalPosition;
    private MovementManner movementManner;
    private TurnManner turnManner;
    
    /**
     * Constructs an {@link AutonDrive} command given an {@link AutoSwerveDrive} subsystem instance,
     * a {@link Position} final position on the field, a {@link MovementManner} describing how the robot
     * should strafe, and a {@link TurnManner} describing how the robot should turn.
     * @param swerveDrive       A {@code AutoSwerveDrive} swerve drive subsystem.
     * @param finalPosition     A final position for the robot on the field. Exactly where this will practically
     * be will depend on how the robot's odometry was last reset with {@link AutoSwerveDrive#resetPosition(Position)}.
     * @param movementManner    A {@code MovementManner} object describing how the robot should strafe autonomously.
     * @param turnManner        A {@code TurnManner} object describing how the robot should turn autonomously.
     */
    public AutonDrive (
            AutoSwerveDrive swerveDrive,
            Position finalPosition,
            MovementManner movementManner,
            TurnManner turnManner) {
        // finalPosition, movementManner, and turnManner can be used in the constructor method
        // but later on they'll be converted to robotMovement and robotTurn
        this.finalPosition = finalPosition;
        this.movementManner = movementManner;
        this.turnManner = turnManner;
        
        this.swerveDrive = swerveDrive;
        
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize () {
        swerveDrive.stop();
        initialPosition = swerveDrive.getPosition(); // used in convertConstructorInputs
        convertConstructorInputs(); // converts constructor inputs from option 1 to option 2 and vice versa
    }
    
    // finalPosition, movementManner, and turnManner can be used in the constructor method
    // this converts them to robotMovement and robotTurn
    private void convertConstructorInputs () {
        if (robotMovement == null) {
            // We have robotMovement and robotTurn but want finalPosition, movementManner, and turnManner
            finalPosition = initialPosition.addMovementVector(robotMovement.toFieldRel(initialPosition));
            movementManner = robotMovement.getManner();
            turnManner = robotTurn.getManner();
        } else {
            // We have finalPosition, movementManner, and turnManner but want robotMovement and robotTurn
            robotMovement = new RobotMovement(initialPosition.movementTo(finalPosition), FrameOfReference.FIELD, movementManner);
            robotTurn = new RobotTurn(finalPosition.getDirection(), FrameOfReference.FIELD, turnManner);
        }
        
        // If we already have robotMovement and robotTurn, then no conversion is necessary
        if (robotMovement != null) return;
        
        // We have finalPosition, movementManner and turnManner and we need robotMovement and robotTurn
        robotMovement = new RobotMovement(initialPosition.movementTo(finalPosition), FrameOfReference.FIELD, movementManner);
        robotTurn = new RobotTurn(finalPosition.getDirection(), FrameOfReference.FIELD, turnManner);
    }
    
    @Override
    public void execute () {
        final Position currentPosition = swerveDrive.getPosition();
        
        // Drive the robot
        final Vector movementVector = getMovementVector(currentPosition);
        final double turnSpeed = getTurnSpeed(currentPosition);
        swerveDrive.autoDrive(movementVector.getX(), movementVector.getY(), turnSpeed);
        
        // Update whether the command is finished
        finished = isMovementFinished(currentPosition) && isTurnFinished(currentPosition);
    }
    
    /**
     * Gets the movement vector at a given point in time (every time execute() is called)
     */
    private Vector getMovementVector (Position currentPosition) {
        // These 2 vectors are field relative:
        final Vector totalMovement = initialPosition.movementTo(finalPosition);
        final Vector remainingMovement = currentPosition.movementTo(finalPosition);
        
        // This is the speed we should move at according to the movementManner's speed supplier
        final double speed = movementManner.getSpeedSupplier().getSpeed(
            totalMovement.getMagnitude(),
            remainingMovement.getMagnitude());
        
        if (isMovementFinished(currentPosition)) {
            return Vector.ZERO;
        } else {
            // The movement vector for this frame (but field relative)
            // "speed / getMagnitude" scales fieldRelMove to make its magnitude equal to speed
            Vector fieldRelMove = remainingMovement.scale(speed / remainingMovement.getMagnitude());
            
            // Get the direction relative to the robot to move in
            double robotRelMoveDir = fieldRelMove.getRotationDegrees() - currentPosition.getDirection();
            
            // Convert field relative move to robot relative, preserving magnitude
            return fieldRelMove.toRotationDegrees(robotRelMoveDir);
        }
    }
    
    /**
     * Gets the turn speed at a given point in time (every time execute() is called)
     */
    private double getTurnSpeed (Position currentPosition) {
        // Gets the total turn from initial to final direction on the interval [-180, 180]
        // TODO: There will be a bug here if we try to turn 170 deg to the right, let's say, and begin by turning 20 deg to the left
        // that is: final direction = 170       initial direction = 0       current direction = -20
        // (one of wrapDegreesZeroCenter will be negative)
        final double totalTurn = Angles.wrapDegreesZeroCenter(finalPosition.getDirection() - initialPosition.getDirection());
        final double remainingTurn = Angles.wrapDegreesZeroCenter(finalPosition.getDirection() - currentPosition.getDirection());
        
        // This is the speed we should turn at according to the turnManner's speed supplier
        final double speed = turnManner.getSpeedSupplier().getSpeed(totalTurn, remainingTurn);
        
        if (isTurnFinished(currentPosition)) return 0;
        else return speed;
    }
    
    /**
     * Returns true if the movement is finished based on movementManner.getMarginOfError(), false otherwise
     */
    private boolean isMovementFinished (Position currentPosition) {
        return currentPosition.distanceFrom(finalPosition) < movementManner.getMarginOfError();
    }
    
    /**
     * Returns true if the turn is finished based on turnManner.getMarginOfError(), false otherwise
     */
    private boolean isTurnFinished (Position currentPosition) {
        final double err = Math.abs(Angles.wrapDegreesZeroCenter(currentPosition.getDirection() - finalPosition.getDirection()));
        return err < turnManner.getMarginOfError();
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