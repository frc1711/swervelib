package frc.team1711.swerve.util.odometry;

/**
 * {@code MovementManner} describes the way in which the robot moves along a given {@link RobotMovement}
 * path.
 * @author Gabriel Seaver
 * 
 * @see TurnManner
 */
public class MovementManner extends Manner {
    
    /**
     * Constructs a {@link MovementManner} instance, describing the manner in which the robot
     * should complete a given {@link RobotMovement} path.
     * @param marginOfError A {@code double} describing how far the robot can be from its endpoint in
     * order for the path to be considered finished, in inches.
     * @param speedSupplier A {@link MovementSpeedSupplier} which provides the speed
     * the robot should move at given the total distance of the path and the remaining distance to travel.
     */
    public MovementManner (double marginOfError, MovementSpeedSupplier speedSupplier) {
        super(marginOfError, speedSupplier);
    }
    
    /**
     * Gets the {@link MovementSpeedSupplier} associated with this {@link MovementManner} instance.
     * @return The {@code MovementSpeedSupplier} which describes how quickly the robot
     * should move while following any given {@code RobotMovement} path.
     */
    @Override
    public MovementSpeedSupplier getSpeedSupplier () {
        return (MovementSpeedSupplier)super.getSpeedSupplier();
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
     * A functional interface which gets the speed the robot should move at given the total
     * distance of a {@link RobotMovement} path and the distance that has yet to be traveled. An instance
     * of this functional interface can be accessed from a given {@code RobotMovement} called
     * {@code movement} through the following code: {@code movement.getManner().getSpeedSupplier()}.
     * 
     * @see TurnManner.TurnSpeedSupplier
     */
    @FunctionalInterface
    public static interface MovementSpeedSupplier extends SpeedSupplier {
        
        /**
         * Gets the speed the robot should move at given the total distance of a {@link RobotMovement} path,
         * in inches, and the distance already traveled.
         * @param totalPathDistance The total distance of the {@code RobotMovement} path, in inches.
         * @param remainingDistance The distance from the robot to the endpoint of the path, in inches.
         * @return The speed the robot should move at, on the interval [0, 1].
         */
        @Override
        public double getSpeed (double totalPathDistance, double remainingDistance);
        
        /**
         * Returns a {@link MovementManner.SpeedSupplier} representing a constant speed at any point
         * in the {@link RobotMovement} path.
         * @param speed The constant robot speed along the path.
         * @return The equivalent {@code MovementManner.SpeedSupplier}.
         */
        public static MovementSpeedSupplier constantSpeed (double speed) {
            return (MovementSpeedSupplier)SpeedSupplier.constantSpeed(speed);
        }
        
        /**
         * Returns a {@link MovementManner.SpeedSupplier} representing a proportional speed to the
         * distance remaining along the {@link RobotMovement} path.
         * @param speedScalar   The scalar for the proportional distance of the path remaining. For example,
         * if 50% of the path is completed and {@code speedScalar} is {@code 0.8}, then the speed {@code 0.4}
         * will be returned. As a result, {@code speedScalar} is both the maximum speed the robot will move along
         * the path and the speed the robot will move along the path when it hasn't yet moved from its starting point.
         * @param minSpeed      The minimum speed the robot will move along the path. If the robot is very near
         * the end of the path, the proportional speed may be too slow for the robot to move to complete the path.
         * {@code minSpeed} solves this problem: if the speed obtained from the proportion of the path remaining
         * multiplied by the {@code speedScalar} is slower than {@code minSpeed}, the robot will just move at
         * a speed of {@code minSpeed}.
         * @return              The equivalent {@code MovementManner.SpeedSupplier}.
         */
        public static MovementSpeedSupplier proportionalSpeed (double speedScalar, double minSpeed) {
            return (MovementSpeedSupplier)SpeedSupplier.proportionalSpeed(speedScalar, minSpeed);
        }
    }
    
}