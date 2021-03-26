// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;
import frc.team1711.swerve.util.Vector;

/**
 * A command which drives a given {@link frc.team1711.swerve.subsystems.AutoSwerveDrive} in any
 * direction, without turning.
 * @see #fromMovement(AutoSwerveDrive, double, double, double, double, double, FrameOfReference)
 * @author Gabriel Seaver
 */
public class AutonDrive extends SequentialCommandGroup {

    private final AutoSwerveDrive swerveDrive;

    private final FrameOfReference frameOfReference;

    // Because directionInput is not based on the gyro, but the robot relative direction
    // may be depending on the frame of reference, they must be stored separately b/c
    // the gyro should not be accessed in the constructor in case the object is initialized
    // before the command is actually used
    private final double
            wheelMarginOfError,
            correctionScalar,
            directionInput,
            distance,
            speed;
    
    // Ease-out mode settings (optional)
    private double
            easeOutDistance,
            easeOutSpeed;

    /**
     * Constructs an {@code AutonDrive} command.
     * @param swerveDrive           The {@link AutoSwerveDrive} drive train.
     * @param direction             The direction, in degrees, to travel in. Zero degrees corresponds with
     * directly forward, and an increase in {@code direction} corresponds with a direction further
     * clockwise from a top-down view. This value must be on the interval [0, 360).
     * @param distance              The distance to travel in the specified direction, in inches. This value
     * must be on the interval (0, infinity).
     * @param speed                 The speed to travel at. This value must be on the interval (0, 1].
     * @param wheelMarginOfError    The acceptable margin of error for each wheel, in degrees, away from
     * their target steering directions before the robot starts actually moving.
     * @param correctionScalar      The speed to turn at per degree offset from the intended direction. For
     * example, if the robot is going ten degrees to the right of where it wants to, it will make a
     * correction turn to the left at a speed of {@code 10*correctionScalar}. A recommended starting value
     * is {@code 0.01}.
     * @param frameOfReference      The {@link FrameOfReference} for this autonomous command.
     */
    public AutonDrive (
            AutoSwerveDrive swerveDrive,
            double direction,
            double distance,
            double speed,
            double wheelMarginOfError,
            double correctionScalar,
            FrameOfReference frameOfReference) {
        this.swerveDrive = swerveDrive;
        this.distance = distance;
        this.speed = speed;
        this.wheelMarginOfError = wheelMarginOfError;
        this.correctionScalar = correctionScalar;
        this.frameOfReference = frameOfReference;
        directionInput = direction;
    }

    // This code cannot be called in constructor because gyro angle may change
    // between object initialization and when the command is first called
    @Override
    public void initialize () {
        double direction = directionInput;
        if (frameOfReference == FrameOfReference.FIELD) direction -= swerveDrive.getGyroAngle();

        if (easeOutDistance != 0) {
            // Ease-out mode is set
            addCommands(
                    new AutonWheelTurn(swerveDrive, direction, wheelMarginOfError),
                    new AutonDriveSimple(swerveDrive, direction, distance-easeOutDistance, speed, correctionScalar),
                    new AutonDriveSimple(swerveDrive, direction, easeOutDistance, easeOutSpeed, correctionScalar));
        } else {
            // No ease-out mode set
            addCommands(
                    new AutonWheelTurn(swerveDrive, direction, wheelMarginOfError),
                    new AutonDriveSimple(swerveDrive, direction, distance, speed, correctionScalar));
        }
        super.initialize();
    }

    /**
     * Adds an ease-out phase during which there will be a change in speed a certain
     * distance before the destination.
     * @param distanceUntilFinish   The distance, in inches, before the target destination, when
     * ease-out mode begins.
     * @param easeOutSpeed          The speed during the ease-out phase.
     */
    public void setEaseOut (double distanceUntilFinish, double easeOutSpeed) {
        easeOutDistance = distanceUntilFinish;
        this.easeOutSpeed = easeOutSpeed;
    }
    
    /**
     * Constructs an {@code AutonDrive} command from a desired distance to the right and forwards.
     * @param swerveDrive       The {@link AutoSwerveDrive} drive train.
     * @param inchesRight       The number of inches to move to the right, where a negative value denotes
     * movement to the left.
     * @param inchesForward     The number of inches to move forwards, where a negative value denotes
     * movement backwards.
     * @param speed             The speed to travel at. This value must be on the interval (0, 1].
     * @param wheelMarginOfError    The acceptable margin of error for each wheel, in degrees, away from
     * their target steering directions before the robot starts actually moving.
     * @param correctionScalar      The speed to turn at per degree offset from the intended direction. For
     * example, if the robot is going ten degrees to the right of where it wants to, it will make a
     * correction turn to the left at a speed of {@code 10*correctionScalar}. A recommended starting value
     * is {@code 0.01}.
     * @param frameOfReference  The {@link FrameOfReference} for this autonomous command. Note that a
     * frame of reference relative to the field does not mean that {@code inchesRight} and
     * {@code inchesForward} are relative to the initial position of the gyro: only steering is relative
     * to the gyro's initial orientation.
     * @return                  The {@code AutonDrive}
     */
    public static AutonDrive fromMovement (
                AutoSwerveDrive swerveDrive,
                double inchesRight,
                double inchesForward,
                double speed,
                double wheelMarginOfError,
                double correctionScalar,
                FrameOfReference frameOfReference) {
        final Vector moveVector = new Vector(inchesRight, inchesForward);
        return new AutonDrive(
                swerveDrive,
                moveVector.getRotationDegrees(),
                moveVector.getMagnitude(),
                speed,
                wheelMarginOfError,
                correctionScalar,
                frameOfReference);
    }
    
}