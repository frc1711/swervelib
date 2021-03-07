// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1711.swerve.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team1711.swerve.util.Vector;

/**
 * Utilizes {@link SwerveWheel} subsystems to create a singular, easy-to-use swerve drive.
 * @author Gabriel Seaver
 */
public class SwerveDrive extends SubsystemBase {
    
    protected final SwerveWheel
            flWheel,
            frWheel,
            rlWheel,
            rrWheel;
    
    /**
     * Default steering speed input scalar.
     * @see #setSteerRelativeSpeed(double)
     * @see #driveRelativeSpeedDefault
     */
    public static double steerRelativeSpeedDefault = 0.3;
    
    /**
     * Default driving speed input scalar.
     * @see #setDriveRelativeSpeed(double)
     * @see #steerRelativeSpeedDefault
     */
    public static double driveRelativeSpeedDefault = 0.5;
    
    protected double
            steerRelativeSpeed,
            driveRelativeSpeed,
            maxOutput,
            deadband;
    
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
        
        driveRelativeSpeed = driveRelativeSpeedDefault;
        steerRelativeSpeed = steerRelativeSpeedDefault;
        
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
        
        // Calculating vectors
        Vector baseVector = new Vector(strafeX * driveRelativeSpeed, strafeY * driveRelativeSpeed);
        if (accountForDeadband(baseVector.getMagnitude()) == 0) baseVector = new Vector(0, 0);
        
        // Steering vector FR is the steering vector that will be added to the FR wheel
        steering = accountForDeadband(steering);
        final Vector steeringVectorFR = new Vector(steering * widthToHeightRatio * steerRelativeSpeed, -steering * steerRelativeSpeed);
        
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
        
        if (maxSpeed > maxOutput) {
            flSpeed /= maxSpeed;
            frSpeed /= maxSpeed;
            rlSpeed /= maxSpeed;
            rrSpeed /= maxSpeed;
            
            flSpeed *= maxOutput;
            frSpeed *= maxOutput;
            rlSpeed *= maxOutput;
            rrSpeed *= maxOutput;
        }
        
        // Vectors default to 90 degrees; no direction change if there's no input
        double flDirection = flVector.getMagnitude() > 0 ? flVector.getRotationDegrees() : flWheel.getDirection();
        double frDirection = flVector.getMagnitude() > 0 ? frVector.getRotationDegrees() : frWheel.getDirection();
        double rlDirection = flVector.getMagnitude() > 0 ? rlVector.getRotationDegrees() : rlWheel.getDirection();
        double rrDirection = flVector.getMagnitude() > 0 ? rrVector.getRotationDegrees() : rrWheel.getDirection();
        
        // Sets the final wheel speeds and rotations
        flWheel.steerAndDrive(flDirection, flSpeed);
        frWheel.steerAndDrive(frDirection, frSpeed);
        rlWheel.steerAndDrive(rlDirection, rlSpeed);
        rrWheel.steerAndDrive(rrDirection, rrSpeed);
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
        
        // TODO: Maybe make a system print for each one of these conditions individually
        // to see why it fails
        return  flWheel.checkWithin180Range(direction, marginOfError) &&
                frWheel.checkWithin180Range(direction, marginOfError) &&
                rlWheel.checkWithin180Range(direction, marginOfError) &&
                rrWheel.checkWithin180Range(direction, marginOfError);
    }
    
    /**
     * Stops all modules immediately.
     */
    public void stop () {
        flWheel.stop();
        frWheel.stop();
        rlWheel.stop();
        rrWheel.stop();
    }
    
    /**
     * Sets the maximum possible drive speed of a swerve wheel.
     * @param _maxOutput    The maximum possible drive speed.
     */
    public void setMaxOutput (double _maxOutput) {
        maxOutput = _maxOutput;
    }
    
    /**
     * Sets the input deadband for {@link #inputDrive(double, double, double)} (i.e. sets the minimum input
     * value required for it to count as being a nonzero input).
     * @param _deadband The new input deadband
     */
    public void setDeadband (double _deadband) {
        deadband = _deadband;
    }
    
    /**
     * Sets the sensitivity of {@link #inputDrive(double, double, double)} towards
     * steering inputs.
     * @param _steerRelativeSpeed The new sensitivity
     * @see #steerRelativeSpeedDefault
     */
    public void setSteerRelativeSpeed (double _steerRelativeSpeed) {
        steerRelativeSpeed = _steerRelativeSpeed;
    }
    
    /**
     * Sets the sensitivity of {@link #inputDrive(double, double, double)} towards
     * driving inputs.
     * @param _driveRelativeSpeed The new sensitivity
     * @see #driveRelativeSpeedDefault
     */
    public void setDriveRelativeSpeed (double _driveRelativeSpeed) {
        driveRelativeSpeed = _driveRelativeSpeed;
    }
    
    protected double accountForDeadband (double value) {
        if (Math.abs(value) < deadband) return 0;
        // Puts value in [m_deadband, 1] or [-1, -m_deadband] into range [0, 1] or [-1, 0]
        return (value + (value > 0 ? -deadband : deadband)) / (1 - deadband);
    }
    
}