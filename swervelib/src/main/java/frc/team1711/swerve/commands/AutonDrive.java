// SwerveLib - Written and maintained by First Robotics Competition team 1711 The RAPTORS.
// https://github.com/frc1711/swervelib

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
    
    /**
     * Constructs an {@code AutonDrive} command with an ease-out phase.
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
     * @param easeOutDistance       The distance before the destination at which to enter the ease-out phase.
     * @param easeOutSpeed          The reduced speed during the ease-out phase.
     */
    public AutonDrive (
            AutoSwerveDrive swerveDrive,
            double direction,
            double distance,
            double speed,
            double wheelMarginOfError,
            double correctionScalar,
            FrameOfReference frameOfReference,
            double easeOutSpeed,
            double easeOutDistance) {
        
        super(
            new AutonWheelTurn(swerveDrive, direction, wheelMarginOfError),
            new AutonDriveSimple(swerveDrive, direction, distance - easeOutDistance, speed, correctionScalar, frameOfReference),
            new AutonDriveSimple(swerveDrive, direction, easeOutDistance, easeOutSpeed, correctionScalar, frameOfReference));
    }
    
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
        
        super(
            new AutonWheelTurn(swerveDrive, direction, wheelMarginOfError),
            new AutonDriveSimple(swerveDrive, direction, distance, speed, correctionScalar, frameOfReference));
    }
    
    /**
     * Constructs an {@code AutonDrive} command with an ease-out phase from a desired distance to the right and forwards.
     * @param swerveDrive           The {@link AutoSwerveDrive} drive train.
     * @param inchesRight           The number of inches to move to the right, where a negative value denotes
     * movement to the left.
     * @param inchesForward         The number of inches to move forwards, where a negative value denotes
     * movement backwards.
     * @param speed                 The speed to travel at. This value must be on the interval (0, 1].
     * @param wheelMarginOfError    The acceptable margin of error for each wheel, in degrees, away from
     * their target steering directions before the robot starts actually moving.
     * @param correctionScalar      The speed to turn at per degree offset from the intended direction. For
     * example, if the robot is going ten degrees to the right of where it wants to, it will make a
     * correction turn to the left at a speed of {@code 10*correctionScalar}. A recommended starting value
     * is {@code 0.01}.
     * @param frameOfReference      The {@link FrameOfReference} for this autonomous command. Note that a
     * frame of reference relative to the field does not mean that {@code inchesRight} and
     * {@code inchesForward} are relative to the initial position of the gyro: only steering is relative
     * to the gyro's initial orientation.
     * @param easeOutDistance       The distance before the destination at which to enter the ease-out phase.
     * @param easeOutSpeed          The reduced speed during the ease-out phase.
     * @return                      The {@code AutonDrive}
     */
    public static AutonDrive fromMovement (
                AutoSwerveDrive swerveDrive,
                double inchesRight,
                double inchesForward,
                double speed,
                double wheelMarginOfError,
                double correctionScalar,
                FrameOfReference frameOfReference,
                double easeOutSpeed,
                double easeOutDistance) {
        final Vector moveVector = new Vector(inchesRight, inchesForward);
        
        if (easeOutDistance == 0)
            return new AutonDrive(
                swerveDrive,
                moveVector.getRotationDegrees(),
                moveVector.getMagnitude(),
                speed,
                wheelMarginOfError,
                correctionScalar,
                frameOfReference);
        else
            return new AutonDrive(
                swerveDrive,
                moveVector.getRotationDegrees(),
                moveVector.getMagnitude(),
                speed,
                wheelMarginOfError,
                correctionScalar,
                frameOfReference,
                easeOutSpeed,
                easeOutDistance);
    }
    
    /**
     * Constructs an {@code AutonDrive} command from a desired distance to the right and forwards.
     * @param swerveDrive           The {@link AutoSwerveDrive} drive train.
     * @param inchesRight           The number of inches to move to the right, where a negative value denotes
     * movement to the left.
     * @param inchesForward         The number of inches to move forwards, where a negative value denotes
     * movement backwards.
     * @param speed                 The speed to travel at. This value must be on the interval (0, 1].
     * @param wheelMarginOfError    The acceptable margin of error for each wheel, in degrees, away from
     * their target steering directions before the robot starts actually moving.
     * @param correctionScalar      The speed to turn at per degree offset from the intended direction. For
     * example, if the robot is going ten degrees to the right of where it wants to, it will make a
     * correction turn to the left at a speed of {@code 10*correctionScalar}. A recommended starting value
     * is {@code 0.01}.
     * @param frameOfReference      The {@link FrameOfReference} for this autonomous command. Note that a
     * frame of reference relative to the field does not mean that {@code inchesRight} and
     * {@code inchesForward} are relative to the initial position of the gyro: only steering is relative
     * to the gyro's initial orientation.
     * @return                      The {@code AutonDrive}
     */
    public static AutonDrive fromMovement (
                AutoSwerveDrive swerveDrive,
                double inchesRight,
                double inchesForward,
                double speed,
                double wheelMarginOfError,
                double correctionScalar,
                FrameOfReference frameOfReference) {
        return fromMovement(swerveDrive, inchesRight, inchesForward, speed, wheelMarginOfError, correctionScalar, frameOfReference, 0, 0);
    }
    
}