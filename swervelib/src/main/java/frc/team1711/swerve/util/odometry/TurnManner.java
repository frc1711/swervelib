package frc.team1711.swerve.util.odometry;

/**
 * {@code TurnManner} describes the way in which the robot performs an autonomous {@link RobotTurn}.
 * 
 * @see MovementManner
 */
public class TurnManner extends Manner {
    
    /**
     * Constructs a {@link TurnManner} instance, describing the manner in which the robot
     * should complete a given autonomous {@link RobotTurn}.
     * @param marginOfError A {@code double} describing how far the robot can be from its endpoint in
     * order for the turn to be considered finished, in degrees.
     * @param speedSupplier A {@link TurnSpeedSupplier} which provides the speed
     * the robot should turn at given the total angle measure of the turn and the remaining angle measure.
     */
    public TurnManner (double marginOfError, TurnSpeedSupplier speedSupplier) {
        super(marginOfError, speedSupplier);
    }
    
    /**
     * Gets the {@link TurnSpeedSupplier} associated with this {@link TurnManner} instance.
     * @return The {@code TurnSpeedSupplier} which describes how quickly the robot
     * should turn while following any given autonomous {@link RobotTurn}.
     */
    @Override
    public TurnSpeedSupplier getSpeedSupplier () {
        return (TurnSpeedSupplier)super.getSpeedSupplier();
    }
    
    /**
     * Gets the angular margin of error at the endpoint, such that if the robot is within this
     * angle measure from the endpoint of the autonomous {@link RobotTurn} the turn is considered to be
     * finished.
     * @return The angular margin of error, measured in degrees.
     */
    @Override
    public double getMarginOfError () {
        return super.getMarginOfError();
    }
    
    /**
     * A functional interface which gets the speed the robot should turn at given the total
     * angular measure of an autonomous {@link RobotTurn} path and the remaining angle to turn. An instance
     * of this functional interface can be accessed from a given {@code RobotTurn} called
     * {@code turn} through the following code: {@code turn.getManner().getSpeedSupplier()}.
     * 
     * @see MovementManner.MovementSpeedSupplier
     */
    @FunctionalInterface
    public static interface TurnSpeedSupplier extends SpeedSupplier {
        
        /**
         * Gets the speed the robot should turn at given the total angular measure of an autonomous {@link RobotTurn},
         * in degrees, and the remaining angle to turn.
         * @param totalTurn The absolute value of the total {@code RobotTurn} angle, in degrees.
         * @param remainingTurn The absolute value of the remaining angle for the robot to turn, in degrees.
         * @return The speed the robot should turn at, on the interval [0, 1].
         */
        @Override
        public double getSpeed (double totalTurn, double remainingTurn);
        
        /**
         * Returns a {@link TurnSpeedSupplier} representing a constant turn speed at any point
         * in the autonomous {@link RobotTurn}.
         * @param speed The constant robot turn speed.
         * @return The equivalent {@code TurnSpeedSupplier}.
         */
        public static TurnSpeedSupplier constantSpeed (double speed) {
            return (TurnSpeedSupplier)SpeedSupplier.constantSpeed(speed);
        }
        
        /**
         * Returns a {@link TurnSpeedSupplier} representing a proportional speed to the
         * angle remaining along the autonomous {@link RobotTurn}.
         * @param speedScalar   The scalar for the proportional distance of the path remaining. For example,
         * if 50% of the turn is completed and {@code speedScalar} is {@code 0.8}, then the speed {@code 0.4}
         * will be returned. As a result, {@code speedScalar} is both the maximum speed the robot will turn
         * and the speed the robot will turn when it hasn't yet moved from its starting angle.
         * @param minSpeed      The minimum speed the robot will turn. If the robot is very near
         * the end of the turn, the proportional speed may be too slow for the robot to finish the {@code RobotTurn}.
         * {@code minSpeed} solves this problem: if the speed obtained from the proportion of the turn remaining
         * multiplied by the {@code speedScalar} is slower than {@code minSpeed}, the robot will just turn at
         * a speed of {@code minSpeed}.
         * @return              The equivalent {@code TurnSpeedSupplier}.
         */
        public static TurnSpeedSupplier proportionalSpeed (double speedScalar, double minSpeed) {
            return (TurnSpeedSupplier)SpeedSupplier.proportionalSpeed(speedScalar, minSpeed);
        }
    }
    
}