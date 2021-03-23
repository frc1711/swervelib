// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;

/**
 * Steers the {@link AutoSwerveDrive}'s wheels in a certain direction,
 * relative to the robot.
 * @author Gabriel Seaver
 */
class AutonWheelTurn extends CommandBase {
    
    private static final double MARGIN_OF_ERROR = 8;
    
    private final AutoSwerveDrive swerveDrive;
    
    private boolean finished;
    
    private final double direction;
    
    /**
     * Constructs a new {@code AutonWheelTurn}.
     * @param swerveDrive   The {@link AutoSwerveDrive} drive train
     * @param direction     The direction, in degrees, to steer the wheels in. Zero degrees
     * corresponds with directly forward, and an increase in {@code direction} corresponds with
     * a direction further clockwise from a top-down view. This value must be on the interval [0, 360).
     */
    AutonWheelTurn (AutoSwerveDrive swerveDrive, double direction) {
        this.swerveDrive = swerveDrive;
        this.direction = direction;
        
        finished = false;
        
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize () {
        swerveDrive.stop();
        swerveDrive.setDistanceReference();
    }
    
    @Override
    public void execute () {
        if (swerveDrive.steerAllWithinRange(direction, MARGIN_OF_ERROR)) finished = true;
    }
    
    @Override
    public void end (boolean interrupted) {
        swerveDrive.stop();
    }
    
    @Override
    public boolean isFinished () {
        return finished;
    }
    
}