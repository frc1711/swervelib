// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

package frc.team1711.swerve.util;

/**
 * Represents a system for converting continuous user input into a usable input into robot-control functions.
 * @author Gabriel Seaver
 */
public class InputHandler {
    
    private final double inputDeadband;
    private final Curve inputCurve;
    
    /**
     * Creates a new {@code InputHandler} given a specified deadband and input curve.
     * @param inputDeadband	A minimum input value for which the input should be considered
     * nonzero. The input will be scaled around the deadband such that arbitrarily low outputs
     * can be reached by inputting values just above the deadband. Primarily used to combat
     * stick drift. Should be on the interval [0, 1).
     * @param inputCurve The {@link Curve} used to convert input values (after applying the deadband)
     * to output values.
     */
    public InputHandler (double inputDeadband, Curve inputCurve) {
        this.inputDeadband = inputDeadband;
        this.inputCurve = inputCurve;
    }
    
    /**
     * Creates a new {@code InputHandler} given a specified deadband. Other than the deadband,
     * the mapping of input to output is linear.
     * @param inputDeadband	A minimum input value for which the input should be considered
     * nonzero. The input will be scaled around the deadband such that arbitrarily low outputs
     * can be reached by inputting values just above the deadband. Primarily used to combat
     * stick drift. Should be on the interval [0, 1).
     */
    public InputHandler (double inputDeadband) {
        this(inputDeadband, Curve.linearCurve);
    }
    
    /**
     * Gets the output of the input handler, after applying both the given input deadband and the input curve.
     * @param input The input value, on the interval [-1, 1]
     * @return The corresponding output value, on the interval [-1, 1]
     * 
     * @see #apply(Vector)
     */
    public final double apply (double input) {
        // Makes the input positive, and keeps track of whether it was originally negative
        // because the input curves only accept positive values
        boolean isNegative = input < 0;
        if (isNegative) input = -input;
        
        // Limits the input to a maximum magnitude of 1
        if (input > 1) input = 1;
        
        // Applies the deadband and curve
        input = inputCurve.getOutput(applyDeadband(input));
        
        // Makes the input negative if it was originally negative
        if (isNegative) input = -input;
        
        return input;
    }
    
    /**
     * Maps an input vector according to the mechanics of {@link #apply(double)}, based on
     * the magnitude of the vector.
     * @param input The input vector, with a magnitude on the interval [-1, 1]
     * @return The corresponding output vector, with a magnitude on the interval [-1, 1]
     */
    public final Vector apply (Vector input) {
        // Returns a zero output vector if the input magnitude is zero (prevents dividing by zero later)
        if (input.getMagnitude() == 0) return new Vector(0, 0);
        
        // Gets the new magnitude of the vector based on the deadband and input curve
        double newMag = apply(input.getMagnitude());
        
        // Scales the vector to the new magnitude
        // Could divide by zero if the input magnitude is zero, but prevented by an earlier check
        return input.scale(newMag / input.getMagnitude());
    }
    
    // Applies the deadband to the input value on the interval [0, 1],
    // linearly mapping to outputs on the interval [0, 1]
    private double applyDeadband (double input) {
        // Maps input from [0, 1] to [-deadband, 1-deadband]
        input -= inputDeadband;
        
        // Maps input from [-deadband, 1-deadband] to [0, 1-deadband]
        if (input < 0) input = 0;
        
        // Maps input from [0, 1-deadband] to [0, 1]
        input /= (1 - inputDeadband);
        
        return input;
    }
    
    /**
     * A functional interface which represents an input curve, useful for allowing for better control
     * at certain input magnitudes. Accepts an input on the range [0, 1], returning an output on the range
     * [0, 1]. As a rule, an input of 0 should map to 0, and an input of 1 should map to 1.
     * 
     * @see #linearCurve
     * @see #squareCurve
     * @see #threeHalvesPowerCurve
     */
    @FunctionalInterface
    public interface Curve {
        /**
         * The function which defines the {@code Curve}. The function should map the interval [0, 1]
         * onto itself (i.e. inputs should be between 0 and 1, and outputs should be between 0 and 1).
         * @param input The input value on the interval [0, 1]
         * @return The corresponding output value of the curve, on the interval [0, 1]
         */
        public double getOutput (double input);
        
        /**
         * A linear curve, that maps any input value to itself.
         */
        public static final Curve linearCurve = x -> x;
        
        /**
         * A square curve, that maps any input value x to x^2.
         * The sensitivity of the output at any given point
         * is proportional to the input (i.e. the derivative of
         * the curve at any point (x, y) is proportional to x).
         * This can make the output much easier to control at
         * lower values.
         */
        public static final Curve squareCurve = x -> x*x;
        
        /**
         * An x^1.5 curve. The sensitivity of the output increases
         * with the input, making the output easier to control
         * at lower values.
         */
        public static final Curve threeHalvesPowerCurve = x -> Math.pow(x, 1.5);
    }
}