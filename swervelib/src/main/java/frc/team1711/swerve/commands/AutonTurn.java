// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;
import frc.team1711.swerve.util.Angles;

/**
 * Turns the entire {@link AutoSwerveDrive} body in a certain direction.
 * @author Gabriel Seaver
 */
public class AutonTurn extends CommandBase {
    
    private static final double
            MARGIN_OF_ERROR = 5;
    
    private final AutoSwerveDrive swerveDrive;
    
    private boolean finished;
    
    private final double
            direction,
            turnSpeed;
    
    /**
     * Constructs a new {@code AutonTurn}.
     * @param swerveDrive   The {@link AutoSwerveDrive} drive train
     * @param direction     The direction, in degrees, to turn the body towards. Zero degrees
     * corresponds with directly forward, and an increase in {@code direction} corresponds with
     * a direction further clockwise from a top-down view.
     * @param turnSpeed     The turning speed of the robot. This must be on the interval (0, 1].
     * @param frameOfReference  The {@link FrameOfReference} for this autonomous command.
     */
    public AutonTurn (AutoSwerveDrive swerveDrive, double direction, double turnSpeed, FrameOfReference frameOfReference) {
        if (turnSpeed > 1 || turnSpeed <= 0) throw new IllegalArgumentException("turnSpeed must be on interval (0, 1]");
        
        this.swerveDrive = swerveDrive;
        
        if (frameOfReference == FrameOfReference.ROBOT) direction += swerveDrive.getGyroAngle();
        this.direction = direction;
        this.turnSpeed = turnSpeed;
        
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
        steerTowardsTarget();
        finished = withinMargins();
    }
    
    private void steerTowardsTarget () {
        // Gets desired change in direction on interval [-180, 180)
        double turningChange = Angles.wrapDegreesZeroCenter(direction - swerveDrive.getGyroAngle());
        double currentTurnSpeed = turningChange > 0 ? turnSpeed : -turnSpeed;
        
        swerveDrive.inputDrive(0, 0, currentTurnSpeed, false);
    }
    
    private boolean withinMargins () {
        // Gets the current direction offset from target direction
        double offset = Angles.wrapDegreesZeroCenter(direction - swerveDrive.getGyroAngle());
        return Math.abs(offset) <= MARGIN_OF_ERROR;
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