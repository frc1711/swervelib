package frc.team1711.swerve.util.odometry;

/**
 * An abstract class representing the way in which an autonomous function is performed by the robot.
 * @author Gabriel Seaver
 * 
 * @see MovementManner
 * @see TurnManner
 */
public abstract class Manner {
    
    private final double marginOfError;
    private final SpeedSupplier speedSupplier;
    
    /**
     * Represents the abstract creation of a manner, which includes a property describing the acceptible margin
     * of error from the endpoint of the auton, and a functional interface describing the speed at which the robot
     * should perform the autonomous activity given certain inputs.
     * @param marginOfError The margin of error from the endpoint of the autonomous activity
     * @param speedSupplier A {@link SpeedSupplier} describing the speed at which to perform the activity
     */
    public Manner (double marginOfError, SpeedSupplier speedSupplier) {
        this.marginOfError = marginOfError;
        this.speedSupplier = speedSupplier;
    }
    
    /**
     * Gets the {@link SpeedSupplier} associated with this {@link Manner}.
     * @return A {@code SpeedSupplier}
     */
    public SpeedSupplier getSpeedSupplier () {
        return speedSupplier;
    }
    
    /**
     * Gets the margin of error around the endpoint, such that if the robot's position is within
     * this margin of error the autonomous activity will be considered complete.
     * @return The margin of error associated with this {@link Manner}
     */
    public double getMarginOfError () {
        return marginOfError;
    }
    
    /**
     * A functional interface which gets the speed the robot should perform an abstract autonomous task given
     * the total amount to be completed (in arbitrary units) and the amount left to be completed.
     * 
     * <p><b>IMPLEMENTATION NOTE: The interfaces which extend {@code SpeedSupplier} do not actually make changes,
     * but are specific to {@link MovementManner} or {@link TurnManner} for clarity. Because of this,
     * any {@code SpeedSupplier} class can be casted to any other class which extends {@code SpeedSupplier}.</b></p>
     * 
     * @see MovementManner.MovementSpeedSupplier
     * @see TurnManner.TurnSpeedSupplier
     */
    @FunctionalInterface
    public static interface SpeedSupplier {
        
        /**
         * Gets the speed the robot should perform the autonomous task at.
         * @param total         The total amount to be completed, in an arbitrary unit.
         * @param remaining     The amount already completed.
         * @return The speed the robot should perform the task at, on the interval [0, 1].
         */
        public double getSpeed (double total, double remaining);
        
        /**
         * Returns a {@link SpeedSupplier} representing a constant speed at all points in the
         * given autonomous task.
         * @param speed The constant robot speed along the path.
         * @return The equivalent {@code Manner.SpeedSupplier}.
         */
        public static SpeedSupplier constantSpeed (double speed) {
            return (a, b) -> speed;
        }
        
        /**
         * Returns a {@link Manner.SpeedSupplier} representing a proportional speed to the
         * amount remaining in the autonomous path.
         * @param speedScalar   The scalar for the proportion of the autonomous task remaining. For example,
         * if 50% of the path is completed and {@code speedScalar} is {@code 0.8}, then the speed {@code 0.4}
         * will be returned. As a result, {@code speedScalar} is both the maximum speed the robot will complete the task at
         * and the speed the robot will complete the task at when it hasn't yet begun.
         * @param minSpeed      The minimum speed the robot will complete the task at. If the robot is very near
         * the end of the path, the proportional speed may be too slow for the robot to finish.
         * {@code minSpeed} solves this problem: if the speed obtained from the proportion of the path remaining
         * multiplied by the {@code speedScalar} is slower than {@code minSpeed}, the robot will just work at
         * a speed of {@code minSpeed}.
         * @return              The equivalent {@code Manner.SpeedSupplier}.
         */
        public static SpeedSupplier proportionalSpeed (double speedScalar, double minSpeed) {
            return (total, remaining) -> {
                // Gets the proportion of the path left to complete, with a maximum of
                // 1 in case the robot started the path in slightly the wrong direction
                final double propLeft = Math.min(remaining / total, 1);
                
                // Returns the proportion remaining * the speed scalar, but if the
                // speed is less than the given minimum speed then the minimum speed
                // is returned instead
                return Math.max(propLeft * speedScalar, minSpeed);
            };
        }
    }
    
}