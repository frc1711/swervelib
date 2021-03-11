// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;

/**
 * A command which drives a given {@link frc.team1711.swerve.subsystems.AutoSwerveDrive} in any
 * direction, without turning.
 * <b>TODO: Still a work in progress.
 * </b>
 * @author Gabriel Seaver
 */
public class AutonDrive extends CommandBase {
    
    private static final double DIRECTION_MARGIN_OF_ERROR = 5;
    private static enum Phases {
        STEERING,
        DRIVING,
        ENDING,
    }
    
    private final AutoSwerveDrive swerveDrive;
    private Phases phase;
    private final double
            direction,
            distance,
            speed;
    
    /**
     * Strafes the swerve drive in a certain direction, at a certain speed, over a certain distance.
     * @param swerveDrive   The {@link AutoSwerveDrive} drive train
     * @param direction     The direction, in degrees, to travel in. Zero degrees corresponds with
     * directly forward, and an increase in {@code direction} corresponds with a direction further
     * clockwise from a top-down view. This value must be on the interval [0, 360).
     * @param distance      The distance to travel in the specified direction, in inches. This value
     * must be on the interval (0, infinity).
     * @param speed         The speed to travel at. This value must be on the interval (0, 1].
     */
    public AutonDrive (AutoSwerveDrive swerveDrive, double direction, double distance, double speed) {
        this.swerveDrive = swerveDrive;
        this.direction = direction;
        this.distance = distance;
        this.speed = speed;
        phase = Phases.DRIVING;
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize () {
        swerveDrive.stop();
    }
    
    @Override
    public void execute () {
        if (phase == Phases.STEERING) executeSteering();
        else if (phase == Phases.DRIVING) executeDriving();
    }
    
    private void executeSteering () {
        // TODO: Debug this (maybe a system print --- you've tested it and the degree check functions work properly so maybe margin is too low)
        if (swerveDrive.steerAllWithinRange(direction, DIRECTION_MARGIN_OF_ERROR)) {
            swerveDrive.setDistanceReference();
            phase = Phases.DRIVING;
        }
    }
    
    private void executeDriving () {
        if (swerveDrive.getDistanceTraveled() < distance) {
            swerveDrive.steerAndDriveAll(direction, speed);
        } else {
            phase = Phases.ENDING;
        }
    }
    
    @Override
    public void end (boolean interrupted) {
        swerveDrive.stop();
    }
    
    @Override
    public boolean isFinished () {
        return phase == Phases.ENDING;
    }
    
}