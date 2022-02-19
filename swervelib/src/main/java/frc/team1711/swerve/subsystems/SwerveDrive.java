// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team1711.swerve.util.InputHandler;
import frc.team1711.swerve.util.Vector;

/**
 * Utilizes {@link SwerveWheel} subsystems to create a singular, easy-to-use swerve drive.
 * @author Gabriel Seaver
 */
public class SwerveDrive extends SubsystemBase {
    
    private final SwerveWheel
		flWheel,
		frWheel,
		rlWheel,
		rrWheel;
    
    protected SwerveDrivingSpeeds swerveDrivingSpeeds;
    
    private final double wheelbaseToTrackRatio;
    
    /**
     * Creates a new {@code SwerveDrive}.
     * @param flWheel					The front left {@code SwerveWheel}
     * @param frWheel					The front right {@code SwerveWheel}
     * @param rlWheel					The rear left {@code SwerveWheel}
     * @param rrWheel					The rear right {@code SwerveWheel}
     * @param wheelbaseToTrackRatio		The distance between the centers of the left and right wheels divided
	 * by the distance between the centers of the front and back wheels
	 * @param swerveDrivingSpeeds		The {@link SwerveDrivingSpeeds} configuration
     */
    public SwerveDrive (
        SwerveWheel flWheel,
        SwerveWheel frWheel,
        SwerveWheel rlWheel,
        SwerveWheel rrWheel,
        double wheelbaseToTrackRatio,
		SwerveDrivingSpeeds swerveDrivingSpeeds) {
        
        this.flWheel = flWheel;
        this.frWheel = frWheel;
        this.rlWheel = rlWheel;
        this.rrWheel = rrWheel;
        this.wheelbaseToTrackRatio = wheelbaseToTrackRatio;
        this.swerveDrivingSpeeds = swerveDrivingSpeeds;
    }
	
	/**
	 * A class representing the configuration of relative speeds for {@link SwerveDrive} in
	 * {@link SwerveDrive#userInputDrive(double, double, double, InputHandler)}.
	 * @see SwerveDrive#setSwerveDrivingSpeeds(SwerveDrivingSpeeds)
	 */
	public static class SwerveDrivingSpeeds {
		
		public final double strafeSpeed, steerSpeed;
		
		/**
		 * Creates a new {@code SwerveDrivingSpeeds} configuration. Note: If {@code strafeSpeed} and {@code steerSpeed} sum to greater
		 * than 1, {@link SwerveDrive#userInputDrive(double, double, double, InputHandler)} will ensure the relative speeds of the modules
		 * are still in proportion to each other, so the kinematics of swerve will not be affected (though in some situations where the
		 * inputs are very high, the robot may move slower than desired).
		 * @param strafeSpeed	The scalar on the strafe inputs for
		 * {@code SwerveDrive.userInputDrive()}. For example, if {@code strafeSpeed} were
		 * 0.8, then any {@link SwerveWheel} would only ever be commanded to a maximum speed of 0.8
		 * using {@code SwerveWheel.setDriveSpeed()}, assuming there is zero steering input passed into {@code SwerveDrive.userInputDrive()}.
		 * @param steerSpeed	The scalar on the steering inputs for {@code SwerveDrive.userInputDrive()},
		 * working in the same way as {@code strafeSpeed}.
		 */
		public SwerveDrivingSpeeds (double strafeSpeed, double steerSpeed) {
			this.strafeSpeed = strafeSpeed;
			this.steerSpeed = steerSpeed;
		}
		
	}
	
	@Override
	public void initSendable (SendableBuilder builder) {
		builder.setSmartDashboardType("Swerve Drive");
		builder.addDoubleProperty("Front Left Module", () -> getSendableDir(flWheel), (x) -> {});
		builder.addDoubleProperty("Front Right Module", () -> getSendableDir(frWheel), (x) -> {});
		builder.addDoubleProperty("Rear Left Module", () -> getSendableDir(rlWheel), (x) -> {});
		builder.addDoubleProperty("Rear Right Module", () -> getSendableDir(rrWheel), (x) -> {});
	}
	
	// Gets the direction to send to the dashboard for a given SwerveWheel
	private double getSendableDir (SwerveWheel wheel) {
		return (wheel.getDirection() + 90) % 180 - 90;
	}
	
	/**
	 * Used to set a new {@link SwerveDrivingSpeeds} configuration.
	 * @param swerveDrivingSpeeds The new {@code SwerveDrivingSpeeds} configuration
	 */
	public void setSwerveDrivingSpeeds (SwerveDrivingSpeeds swerveDrivingSpeeds) {
		this.swerveDrivingSpeeds = swerveDrivingSpeeds;
	}
	
    /**
     * Drives the {@code SwerveDrive} given strafing and steering inputs, all on the interval [-1, 1],
	 * where +{@code strafeY} is forwards and +{@code strafeX} is to the right. Inputs are assumed to be from a user-controlled
	 * device, so {@link SwerveDrivingSpeeds} are applied, along with an {@link InputHandler}.
     * @param strafeX           The strafing speed in the x direction
     * @param strafeY           The strafing speed in the y direction
     * @param steering          The steering speed, where a positive value steers clockwise from a top-down point of view
     * @param inputHandler		The {@code InputHandler} to be used for converting user inputs into usable outputs.
     * @see #steerAndDriveAll(double, double)
     */
    public final void userInputDrive (double strafeX, double strafeY, double steering, InputHandler inputHandler) {
		
		// Applies inputHandler to all inputs
        Vector strafeVector = new Vector(strafeX, strafeY);
        strafeVector = inputHandler.apply(strafeVector);
        steering = inputHandler.apply(steering);
		
		// Passes new inputs to autoDrive
		autoDrive(
			strafeVector.getX() * swerveDrivingSpeeds.strafeSpeed,
			strafeVector.getY() * swerveDrivingSpeeds.strafeSpeed,
			steering * swerveDrivingSpeeds.steerSpeed);
    }
	
	/**
	 * Drives the {@code SwerveDrive} given strafing and steering inputs, all on the interval [-1, 1],
	 * where +{@code strafeY} is forwards and +{@code strafeX} is to the right. Inputs are assumed to be generated from
	 * a command, so no {@link InputHandler}s or {@link SwerveDrivingSpeeds} are applied to the inputs.
     * @param strafeX           The strafing speed in the x direction
     * @param strafeY           The strafing speed in the y direction
     * @param steering          The steering speed, where a positive value steers clockwise from a top-down point of view
     * @see #steerAndDriveAll(double, double)
	 */
	public final void autoDrive (double strafeX, double strafeY, double steering) {
		// Calculating strafe vector, the vector all the wheels would move at if swerve were to only strafe
        Vector strafeVector = new Vector(strafeX, strafeY);
        
        // Steering vector FR is the steering vector that will be added to the front right wheel, and is used
		// to calculate the steering vectors for all the other wheels
		Vector steeringVectorFR = new Vector(wheelbaseToTrackRatio, -1); // Puts steeringVectorFR in the correct DIRECTION for rotation (wrong magnitude)
		steeringVectorFR = steeringVectorFR.scale(steering/steeringVectorFR.getMagnitude()); // Scales steeringVectorFR to the magnitude of the steering variable
		
        /*
        Clockwise steering vector additions:
        (top-down view of robot with --+ representing vector arrows for clockwise turning)
        See https://www.desmos.com/calculator/3rogeuv7u2
        |
        |        +
        |       /     \
        |   FL   |---| +   FR
        |        |   |
        |  RL  + |---|   RR
        |       \     /
        |            +
        */
        
        // Calculates movement vectors for each wheel, taking into account strafeVector
        // which is how wheels would move if they were to only strafe, and steeringVectorFR
        // which is how the front right wheel were to move if it were to only use the
        // steering input
		// Movement Vector = Strafe Vector + Module's Steering Vector
		// Module's steering vector can be put in terms of a transformation
		// on the front right module's steering vector
        final Vector frVector = strafeVector.add(steeringVectorFR);
        final Vector rrVector = strafeVector.add(steeringVectorFR.reflectAcrossY());
        final Vector rlVector = strafeVector.add(steeringVectorFR.scale(-1));
        final Vector flVector = strafeVector.add(steeringVectorFR.reflectAcrossX());
        
        // Get wheel speeds
        double
			flSpeed = flVector.getMagnitude(),
			frSpeed = frVector.getMagnitude(),
			rlSpeed = rlVector.getMagnitude(),
			rrSpeed = rrVector.getMagnitude();
        
        
        // Because wheel speeds must be in correct proportions in order for swerve
        // to function correctly, we check if the maximum speed is within
        // the proper bounds and if it isn't then divide all by the maximum speed
        final double maxSpeed = Math.max(Math.max(flSpeed, frSpeed), Math.max(rlSpeed, rrSpeed));
        
        if (maxSpeed > 1) {
            flSpeed /= maxSpeed;
            frSpeed /= maxSpeed;
            rlSpeed /= maxSpeed;
            rrSpeed /= maxSpeed;
        }
        
        // Vectors default to 90 degrees; they should not steer in a new direction if they don't need to drive
        double flDirection = flSpeed > 0 ? flVector.getRotationDegrees() : flWheel.getDirection();
        double frDirection = frSpeed > 0 ? frVector.getRotationDegrees() : frWheel.getDirection();
        double rlDirection = rlSpeed > 0 ? rlVector.getRotationDegrees() : rlWheel.getDirection();
        double rrDirection = rrSpeed > 0 ? rrVector.getRotationDegrees() : rrWheel.getDirection();
        
        // Sets the final wheel speeds and rotations
        flWheel.steerAndDrive(flDirection, flSpeed);
        frWheel.steerAndDrive(frDirection, frSpeed);
        rlWheel.steerAndDrive(rlDirection, rlSpeed);
        rrWheel.steerAndDrive(rrDirection, rrSpeed);
	}
    
    /**
     * Steers and drives all wheels in the same direction and with the same speed.
     * {@code targetDirection} must use the system where 0 represents
     * steering directly forwards and an increase represents steering further clockwise.
     * {@code speed} must be on the interval [0, 1], where 1 represents directly
     * forwards.
     * @param direction         The target steering direction
     * @param speed             The speed to drive at
     * @see #userInputDrive(double, double, double, InputHandler)
     */
    public final void steerAndDriveAll (double direction, double speed) {
        flWheel.steerAndDrive(direction, speed);
        frWheel.steerAndDrive(direction, speed);
        rlWheel.steerAndDrive(direction, speed);
        rrWheel.steerAndDrive(direction, speed);
    }
    
    /**
     * Steers all wheels to a direction within a certain range. Direction and margin of error
     * are measured in degrees. The direction should use a system where zero represents
     * directly forward and an increase in direction represents a further clockwise steering
     * direction from a top-down view.
     * @param direction         The target steering direction
     * @param marginOfError     The acceptable margin of error
     * @return A {@code boolean}, which is {@code true} when all wheels are within the range, and
     * {@code false} otherwise.
     */
    public final boolean steerAllWithinRange (double direction, double marginOfError) {
        steerAndDriveAll(direction, 0);
        
        return  flWheel.checkWithin180Range(direction, marginOfError) &&
                frWheel.checkWithin180Range(direction, marginOfError) &&
                rlWheel.checkWithin180Range(direction, marginOfError) &&
                rrWheel.checkWithin180Range(direction, marginOfError);
    }
    
    /**
     * Stops all modules immediately.
     */
    public final void stop () {
        flWheel.stop();
        frWheel.stop();
        rlWheel.stop();
        rrWheel.stop();
    }
    
}