// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

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
    
    /**
     * Represents the abstract creation of a manner, which includes a property describing the acceptible margin
     * of error from the endpoint of the auton, and a functional interface describing the speed at which the robot
     * should perform the autonomous activity given certain inputs.
     * @param marginOfError The margin of error from the endpoint of the autonomous activity
     */
    public Manner (double marginOfError) {
        this.marginOfError = marginOfError;
    }
    
    /**
     * Gets the {@link SpeedSupplier} associated with this {@link Manner}.
     * @return A {@code SpeedSupplier}
     */
    abstract public SpeedSupplier getSpeedSupplier ();
    
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
     * the amount left to be completed.
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
         * @param remaining     The amount already completed, in an arbitrary unit.
         * @return The speed the robot should perform the task at, on the interval [0, 1].
         */
        public double getSpeed (double remaining);
        
        /**
         * Returns a {@link SpeedSupplier} representing a constant speed at all points in the
         * given autonomous task.
         * @param speed The constant robot speed along the path.
         * @return The equivalent {@code Manner.SpeedSupplier}.
         */
        public static SpeedSupplier constantSpeed (double speed) {
            return a -> speed;
        }
        
        /**
         * Returns a {@link Manner.SpeedSupplier} representing a speed that slows down after a certain proximity
         * to the endpoint of the autonomous task, until a minimum speed is hit. That is, this type of
         * {@code Manner.SpeedSupplier} will return a given default (maximum) speed up until the robot must
         * slow down (the slowdown offset). Then, the robot will slow down proportionally after the slowdown offset.
         * However, the robot will not slow down past a given minimum speed.
         * @param maxSpeed          The speed the robot will move/complete the autonomous task at for the majority
         * of the path.
         * @param slowdownOffset    The point (in units for distance, rotation, etc.) from the end position of the
         * autonomous task at which the robot will begin proportionally slowing down.
         * @param minSpeed          After the slowdown offset, the robot will not move any slower than {@code minSpeed}.
         * This is to prevent the robot moving so slowly that friction or some other force prevents the robot from fully
         * completing the autonomous path.
         * @return                  A {@code Manner.SpeedSupplier} described by the given parameters.
         */
        public static SpeedSupplier speedWithSlowdown (double maxSpeed, double slowdownOffset, double minSpeed) {
            return remaining -> {
                // Return the max speed if we haven't hit slowdownOffset yet
                if (remaining > slowdownOffset) return maxSpeed;
                // Return the reduced speed after the slowdown offset (but limited with a minimum of minSpeed)
                return Math.max(maxSpeed * remaining / slowdownOffset, minSpeed);
            };
        }
    }
    
}