// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;

/**
 * Steers the {@link AutoSwerveDrive}'s wheels in a certain direction, relative to the robot.
 * @author Gabriel Seaver
 */
class AutonWheelTurn extends CommandBase {
    
    private final double marginOfError;
    
    private final AutoSwerveDrive swerveDrive;
    
    private boolean finished;
    
    private final double direction;
    
    /**
     * Constructs a new {@code AutonWheelTurn}.
     * @param swerveDrive   The {@link AutoSwerveDrive} drive train
     * @param direction     The direction, in degrees, to steer the wheels in. Zero degrees
     * corresponds with directly forward relative to the robot, and an increase in {@code direction}
     * corresponds with a direction further clockwise from a top-down view.
     * @param marginOfError The acceptable margin of error for each wheel, in degrees, away from
     * their target steering directions.
     */
    AutonWheelTurn (AutoSwerveDrive swerveDrive, double direction, double marginOfError) {
        this.swerveDrive = swerveDrive;
        this.direction = direction;
        this.marginOfError = marginOfError;
        
        finished = false;
        
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize () {
        swerveDrive.stop();
    }
    
    @Override
    public void execute () {
        if (swerveDrive.steerAllWithinRange(direction, marginOfError)) finished = true;
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