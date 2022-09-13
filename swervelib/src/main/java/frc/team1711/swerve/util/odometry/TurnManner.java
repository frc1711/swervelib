package frc.team1711.swerve.util.odometry;

/**
 * {@code TurnManner} describes the way in which the robot performs an autonomous {@link RobotTurn}.
 * @author Gabriel Seaver
 * 
 * @see MovementManner
 */
public class TurnManner extends Manner {
    
    /**
     * A {@code TurnManner} where the robot will not attempt to move and the path will immediately be completed.
     */
    public static final TurnManner NONE = new TurnManner(Double.POSITIVE_INFINITY, TurnSpeedSupplier.constantSpeed(0));
    
    private final TurnSpeedSupplier speedSupplier;
    
    /**
     * Constructs a {@link TurnManner} instance, describing the manner in which the robot
     * should complete a given autonomous {@link RobotTurn}.
     * @param marginOfError A {@code double} describing how far the robot can be from its endpoint in
     * order for the turn to be considered finished, in degrees.
     * @param speedSupplier A {@link TurnSpeedSupplier} which provides the speed
     * the robot should turn at given the remaining angle measure.
     */
    public TurnManner (double marginOfError, TurnSpeedSupplier speedSupplier) {
        super(marginOfError);
        this.speedSupplier = speedSupplier;
    }
    
    /**
     * Gets the {@link TurnSpeedSupplier} associated with this {@link TurnManner} instance.
     * @return The {@code TurnSpeedSupplier} which describes how quickly the robot
     * should turn while following any given autonomous {@link RobotTurn}.
     */
    @Override
    public TurnSpeedSupplier getSpeedSupplier () {
        return speedSupplier;
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
     * A functional interface which gets the speed the robot should turn at given the
     * remaining angle to turn. An instance of this functional interface can be accessed from
     * a given {@code RobotTurn} called {@code turn} through the following code:
     * {@code turn.getManner().getSpeedSupplier()}.
     * 
     * @see MovementManner.MovementSpeedSupplier
     */
    @FunctionalInterface
    public static interface TurnSpeedSupplier extends SpeedSupplier {
        
        /**
         * Gets the speed the robot should turn at given the remaining angle to turn.
         * @param remainingTurn The absolute value of the remaining angle for the robot to turn, in degrees.
         * @return The speed the robot should turn at, on the interval [0, 1].
         */
        @Override
        public double getSpeed (double remainingTurn);
        
        /**
         * Returns a {@link TurnSpeedSupplier} representing a constant turn speed at any point
         * in the autonomous {@link RobotTurn}.
         * @param speed The constant robot turn speed.
         * @return The equivalent {@code TurnSpeedSupplier}.
         */
        public static TurnSpeedSupplier constantSpeed (double speed) {
            return SpeedSupplier.constantSpeed(speed)::getSpeed;
        }
        
        /**
         * Returns a {@link TurnManner.TurnSpeedSupplier} representing a speed that slows down after a certain angular offset
         * from the autonomous direction endpoint, until a minimum speed is hit. That is, this type of
         * {@code TurnManner.TurnSpeedSupplier} will return a given default (maximum) speed up until the robot must
         * slow down (the slowdown angle). Then, when the robot is closer to the endpoint than this angle, the robot will
         * slow down proportionally to the angle from the endpoint. However, the robot will not slow down past a given
         * minimum speed.
         * @param maxSpeed          The speed the robot will move/complete the autonomous turning at for the majority
         * of the turn.
         * @param slowdownAngle     The angle, in degrees, from the endpoint of the autonomous turn at which the robot
         * will begin proportionally slowing down.
         * @param minSpeed          The calculated speed will never be any slower than {@code minSpeed}.
         * This is to prevent the robot turning so slowly after the slowdown angle that friction or some other force
         * prevents the robot from fully completing the autonomous turn.
         * @return                  A {@code TurnManner.TurnSpeedSupplier} described by the given parameters.
         */
        public static TurnSpeedSupplier speedWithSlowdown (double maxSpeed, double slowdownAngle, double minSpeed) {
            return SpeedSupplier.speedWithSlowdown(maxSpeed, slowdownAngle, minSpeed)::getSpeed;
        }
    }
    
}