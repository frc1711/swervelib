// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swerve.drive;

import edu.wpi.first.wpilibj.drive.RobotDriveBase;

import swerve.util.Vector;
import swerve.subsystems.SwerveWheel;

/**
 * Utilizes {@link SwerveWheel} subsystems to create a singular, easy-to-use swerve drive.
 * @author Gabriel Seaver
 */
public class SwerveDrive extends RobotDriveBase {
    
    protected final SwerveWheel
            flWheel,
            frWheel,
            rlWheel,
            rrWheel;
    
    private double
            steerSpeed,
            driveSpeed;
    
    private final double widthToHeightRatio;
    
    /**
     * Creates a new {@code SwerveDrive} given {@link SwerveWheel} wheels.
     * <b>Note: {@link #SwerveDrive(SwerveWheel, SwerveWheel, SwerveWheel, SwerveWheel, double)}
     * should be used instead if the wheelbase and track are not equal.</b>
     * @param _flWheel              The front left {@code SwerveWheel}
     * @param _frWheel              The front right {@code SwerveWheel}
     * @param _rlWheel              The rear left {@code SwerveWheel}
     * @param _rrWheel              The rear right {@code SwerveWheel}
     */
    public SwerveDrive (
        SwerveWheel _flWheel,
        SwerveWheel _frWheel,
        SwerveWheel _rlWheel,
        SwerveWheel _rrWheel) {
        
        this(_flWheel, _frWheel, _rlWheel, _rrWheel, 1);
    }
    
    /**
     * Creates a new {@code SwerveDrive} given {@link SwerveWheel} wheels.
     * @param _flWheel              The front left {@code SwerveWheel}
     * @param _frWheel              The front right {@code SwerveWheel}
     * @param _rlWheel              The rear left {@code SwerveWheel}
     * @param _rrWheel              The rear right {@code SwerveWheel}
     * @param _widthToHeightRatio   The ratio from the track to the wheelbase (the distance between the centers
     * of the front or back wheels divided by the distance between the centers of the left or right wheels).
     * {@link #SwerveDrive(SwerveWheel, SwerveWheel, SwerveWheel, SwerveWheel)} is recommended if this ratio is 1:1.
     */
    public SwerveDrive (
        SwerveWheel _flWheel,
        SwerveWheel _frWheel,
        SwerveWheel _rlWheel,
        SwerveWheel _rrWheel,
        double _widthToHeightRatio) {
        
        flWheel = _flWheel;
        frWheel = _frWheel;
        rlWheel = _rlWheel;
        rrWheel = _rrWheel;
        
        driveSpeed = 0.5 * m_maxOutput;
        steerSpeed = 0.5 * m_maxOutput;
        
        widthToHeightRatio = _widthToHeightRatio;
    }
    
    /**
     * Drives the {@code SwerveDrive} given strafing and steering inputs,
     * all on the interval [-1, 1], where +y is forwards and +x is to the right.
     * @param strafeX       The strafing speed in the x direction
     * @param strafeY       The strafing speed in the y direction
     * @param steering      The steering speed, where a positive value steers clockwise from a top-down point of view
     * @see #steerAndDriveAll(double, double)
     */
    public void inputDrive (double strafeX, double strafeY, double steering) {
        
        // Deadbands
        strafeX = accountForDeadband(strafeX);
        strafeY = accountForDeadband(strafeY);
        steering = accountForDeadband(steering);
        
        // Calculating vectors
        final Vector baseVector = new Vector(strafeX * driveSpeed, strafeY * driveSpeed);
        // Steering vector FR is the steering vector that will be added to the FR wheel
        final Vector steeringVectorFR = new Vector(steering * widthToHeightRatio * steerSpeed, -steering * steerSpeed);
        
        /*
        Clockwise steering vector additions:
        (top-down view of robot with --+ representing vector arrows for clockwise turning)
        See https://www.desmos.com/calculator/3rogeuv7u2
        |
        |        +
        |       /     \
        |   FL   |---| +   FR
        |        |   |
        |  RL  + |---|   RR
        |       \     /
        |            +
        */
        
        final Vector frVector = baseVector.add(steeringVectorFR);
        final Vector rrVector = baseVector.add(steeringVectorFR.reflectAcrossY());
        final Vector rlVector = baseVector.add(steeringVectorFR.scale(-1));
        final Vector flVector = baseVector.add(steeringVectorFR.reflectAcrossX());
        
        // Set wheel speeds
        double
                flSpeed = flVector.getMagnitude(),
                frSpeed = frVector.getMagnitude(),
                rlSpeed = rlVector.getMagnitude(),
                rrSpeed = rrVector.getMagnitude();
        
        
        // Because wheel speeds must be in correct proportions in order for swerve
        // to function correctly, we check if the maximum speed is within
        // the proper bounds and if it isn't then divide all by the maximum speed,
        // then scale to fit the upper limit again.
        final double maxSpeed = Math.max(Math.max(flSpeed, frSpeed), Math.max(rlSpeed, rrSpeed));
        
        if (maxSpeed > m_maxOutput) {
            flSpeed /= maxSpeed;
            frSpeed /= maxSpeed;
            rlSpeed /= maxSpeed;
            rrSpeed /= maxSpeed;
            
            flSpeed *= m_maxOutput;
            frSpeed *= m_maxOutput;
            rlSpeed *= m_maxOutput;
            rrSpeed *= m_maxOutput;
        }
        
        // Sets the final wheel speeds and rotations
        flWheel.steerAndDrive(flVector.getRotationDegrees(), flSpeed);
        frWheel.steerAndDrive(frVector.getRotationDegrees(), frSpeed);
        rlWheel.steerAndDrive(rlVector.getRotationDegrees(), rlSpeed);
        rrWheel.steerAndDrive(rrVector.getRotationDegrees(), rrSpeed);
        
        feed();
    }
    
    /**
     * Steers and drives all wheels in the same direction and with the same speed.
     * {@code targetDirection} must be on the interval [0, 360), where 0 represents
     * steering directly forwards and an increase represents steering further clockwise.
     * {@code speed} must be on the interval [0, 1], where 1 represents directly
     * forwards and -1 represents directly backwards.
     * @param direction         The target steering direction
     * @param speed             The speed to drive at
     * @see #inputDrive(double, double, double)
     */
    public void steerAndDriveAll (double direction, double speed) {
        flWheel.steerAndDrive(direction, speed);
        frWheel.steerAndDrive(direction, speed);
        rlWheel.steerAndDrive(direction, speed);
        rrWheel.steerAndDrive(direction, speed);
        
        feed();
    }
    
    /**
     * Steers all wheels to a direction within a certain range. Direction and margin of error
     * are measured in degrees. The direction should be on the range [0, 360), where zero represents
     * directly forward and an increase in direction represents a further clockwise steering
     * direction from a top-down view.
     * @param direction         The target steering direction
     * @param marginOfError     The acceptable margin of error
     * @return A {@code boolean}, which is {@code true} when all wheels are within the range, and
     * {@code false} otherwise.
     */
    public boolean steerAllWithinRange (double direction, double marginOfError) {
        flWheel.steerAndDrive(direction, 0);
        frWheel.steerAndDrive(direction, 0);
        rlWheel.steerAndDrive(direction, 0);
        rrWheel.steerAndDrive(direction, 0);
        
        feed();
        
        return  flWheel.checkWithin180Range(direction, marginOfError) &&
                frWheel.checkWithin180Range(direction, marginOfError) &&
                rlWheel.checkWithin180Range(direction, marginOfError) &&
                rrWheel.checkWithin180Range(direction, marginOfError);
    }
    
    /**
     * Stops all modules immediately.
     */
    @Override
    public void stopMotor () {
        flWheel.stop();
        frWheel.stop();
        rlWheel.stop();
        rrWheel.stop();
        
        feed();
    }
    
    @Override
    public String getDescription () {
        return "SwerveDrive";
    }
    
    @Override
    public void setMaxOutput (double maxOutput) {
        m_maxOutput = maxOutput;
        driveSpeed = 0.5 * m_maxOutput;
        steerSpeed = 0.5 * m_maxOutput;
    }
    
    private double accountForDeadband (double value) {
        if (Math.abs(value) < m_deadband) return 0;
        // Puts value in [m_deadband, 1] or [-1, -m_deadband] into range [0, 1] or [-1, 0]
        return (value + (value > 0 ? -m_deadband : m_deadband)) / (1 - m_deadband);
    }
    
}
// TODO: Turn into real gradle library: https://docs.gradle.org/current/samples/sample_building_java_libraries.html