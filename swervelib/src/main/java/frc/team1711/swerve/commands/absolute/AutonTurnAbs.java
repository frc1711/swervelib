// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.commands.absolute;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;

/**
 * Turns the entire {@link AutoSwerveDrive} body in a certain direction,
 * relative to the gyro's initialization orientation.
 * @author Gabriel Seaver
 */
public class AutonTurnAbs extends CommandBase {
    
    private static final double
            MARGIN_OF_ERROR = 5,
            STEERING_SPEED_SCALAR = .15,
            MINIMUM_TURN_SPEED = .1;
    
    private final AutoSwerveDrive swerveDrive;
    
    private boolean finished;
    
    private final double direction;
    
    /**
     * Constructs a new {@code AutonTurnAbs}.
     * @param swerveDrive   The {@link AutoSwerveDrive} drive train
     * @param direction     The direction, in degrees, to turn the body towards. Zero degrees
     * corresponds with directly forward relative to the field (that is, the initial gyro
     * orientation), and an increase in {@code direction} corresponds with a direction further
     * clockwise from a top-down view. This value must be on the interval [0, 360).
     */
    public AutonTurnAbs (AutoSwerveDrive swerveDrive, double direction) {
        this.swerveDrive = swerveDrive;
        
        while (direction >= 180) direction -= 360;
        while (direction < -180) direction += 360;
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
        steerTowardsTarget();
        finished = withinMargins();
    }
    
    private void steerTowardsTarget () {
        double steering = direction - swerveDrive.getGyroAngle();
        while (steering >= 180) steering -= 360;
        while (steering < -180) steering += 360;
        steering = Math.sqrt(steering) * STEERING_SPEED_SCALAR;
        System.out.println(steering);
        
        if (steering > 0) steering = Math.max(steering, MINIMUM_TURN_SPEED);
        else steering = Math.min(steering, -MINIMUM_TURN_SPEED);
        
        swerveDrive.inputDrive(0, 0, steering);
    }
    
    private boolean withinMargins () {
        double offset = direction - swerveDrive.getGyroAngle();
        while (offset >= 180) offset -= 360;
        while (offset < -180) offset += 360;
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