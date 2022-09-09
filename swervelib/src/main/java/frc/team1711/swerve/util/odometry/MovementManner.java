package frc.team1711.swerve.util.odometry;

/**
 * {@code MovementManner} describes the way in which the robot moves along a given {@link RobotMovement}
 * path.
 * @author Gabriel Seaver
 * 
 * @see TurnManner
 */
public class MovementManner extends Manner {
    
    private final MovementSpeedSupplier speedSupplier;
    
    /**
     * Constructs a {@link MovementManner} instance, describing the manner in which the robot
     * should complete a given {@link RobotMovement} path.
     * @param marginOfError A {@code double} describing how far the robot can be from its endpoint in
     * order for the path to be considered finished, in inches.
     * @param speedSupplier A {@link MovementSpeedSupplier} which provides the speed
     * the robot should move at given the remaining distance to travel.
     */
    public MovementManner (double marginOfError, MovementSpeedSupplier speedSupplier) {
        super(marginOfError);
        this.speedSupplier = speedSupplier;
    }
    
    /**
     * Gets the {@link MovementSpeedSupplier} associated with this {@link MovementManner} instance.
     * @return The {@code MovementSpeedSupplier} which describes how quickly the robot
     * should move while following any given {@code RobotMovement} path.
     */
    @Override
    public MovementSpeedSupplier getSpeedSupplier () {
        return speedSupplier;
    }
    
    /**
     * Gets the distance margin of error around the endpoint, where if the robot is within this
     * distance from the endpoint of the {@link RobotMovement} path the path is considered to be
     * finished.
     * @return The distance margin of error, measured in inches.
     */
    @Override
    public double getMarginOfError () {
        return super.getMarginOfError();
    }
    
    /**
     * A functional interface which gets the speed the robot should move at given the distance
     * that has yet to be traveled. An instance of this functional interface can be accessed from
     * a given {@code RobotMovement} called {@code movement} through the following code:
     * {@code movement.getManner().getSpeedSupplier()}.
     * 
     * @see TurnManner.TurnSpeedSupplier
     */
    @FunctionalInterface
    public static interface MovementSpeedSupplier extends SpeedSupplier {
        
        /**
         * Gets the speed the robot should move at given the distance yet to be traveled, in inches.
         * @param remainingDistance The distance from the robot to the endpoint of the path, in inches.
         * @return The speed the robot should move at, on the interval [0, 1].
         */
        @Override
        public double getSpeed (double remainingDistance);
        
        /**
         * Returns a {@link MovementManner.SpeedSupplier} representing a constant speed at any point
         * in the {@link RobotMovement} path.
         * @param speed The constant robot speed along the path.
         * @return The equivalent {@code MovementManner.SpeedSupplier}.
         */
        public static MovementSpeedSupplier constantSpeed (double speed) {
            return SpeedSupplier.constantSpeed(speed)::getSpeed;
        }
        
        /**
         * Returns a {@link MovementManner.MovementSpeedSupplier} representing a speed that slows down after a certain distance
         * to the autonomous endpoint, until a minimum speed is hit. That is, this type of
         * {@code MovementManner.MovementSpeedSupplier} will return a given default (maximum) speed up until the robot must
         * slow down (the slowdown distance). Then, when the robot is closer to the endpoint than this distance, the robot will
         * slow down proportionally to the distance from the endpoint. However, the robot will not slow down past a given
         * minimum speed.
         * @param maxSpeed          The speed the robot will move/complete the autonomous driving at for the majority
         * of the path.
         * @param slowdownDistance  The distance, in inches, from the endpoint of the autonomous driving at which the robot
         * will begin proportionally slowing down.
         * @param minSpeed          The calculated speed will never be any slower than {@code minSpeed}.
         * This is to prevent the robot moving so slowly after the slowdown distance that friction or some other force
         * prevents the robot from fully completing the autonomous path.
         * @return                  A {@code MovementManner.MovementSpeedSupplier} described by the given parameters.
         */
        public static MovementSpeedSupplier speedWithSlowdown (double maxSpeed, double slowdownDistance, double minSpeed) {
            return SpeedSupplier.speedWithSlowdown(maxSpeed, slowdownDistance, minSpeed)::getSpeed;
        }
    }
    
}