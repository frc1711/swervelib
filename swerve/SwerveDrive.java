// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swerve;

import edu.wpi.first.wpilibj.drive.RobotDriveBase;

/**
 * Utilizes {@link SwerveWheel} subsystems to create a singular, easy-to-use swerve drive.
 * @author Gabriel Seaver
 */
public class SwerveDrive extends RobotDriveBase {
    
    private final SwerveWheel
            flWheel,
            frWheel,
            rlWheel,
            rrWheel;
    
    private final double
            maxWheelSpeed,
            turnSpeed,
            directMoveSpeed,
            widthToHeightRatio;
    
    /**
     * Creates a new {@code SwerveDrive} given {@link SwerveWheel} wheels and several speed constants.
     * <b>Note: {@link #SwerveDrive(SwerveWheel, SwerveWheel, SwerveWheel, SwerveWheel, double, double,
     * double, double)} should be used instead if the wheelbase is not a square.</b>
     * @param _flWheel              The front left {@code SwerveWheel}
     * @param _frWheel              The front right {@code SwerveWheel}
     * @param _rlWheel              The rear left {@code SwerveWheel}
     * @param _rrWheel              The rear right {@code SwerveWheel}
     * @param _maxWheelSpeed        The maximum directional speed that a {@code SwerveWheel} can be set to
     * @param _turnSpeed            A value deciding how quickly the robot will turn, relative to {@code
     * _directMoveSpeed}. This should be adjusted over time while testing. A recommended initial value is
     * 1/2 of {@code _maxWheelSpeed}.
     * @param _directMoveSpeed      A value deciding how quickly the robot will strafe, relative to {@code
     * _turnSpeed}. This should be adjusted over time while testing. A recommended initial value is
     * 1/2 of {@code _maxWheelSpeed}.
     */
    public SwerveDrive (
        SwerveWheel _flWheel,
        SwerveWheel _frWheel,
        SwerveWheel _rlWheel,
        SwerveWheel _rrWheel,
        double _maxWheelSpeed,
        double _turnSpeed,
        double _directMoveSpeed) {
        
        this(_flWheel, _frWheel, _rlWheel, _rrWheel, _maxWheelSpeed, _turnSpeed, _directMoveSpeed, 1);
    }
    
    /**
     * Creates a new {@code SwerveDrive} given {@link SwerveWheel} wheels and several speed constants.
     * @param _flWheel              The front left {@code SwerveWheel}
     * @param _frWheel              The front right {@code SwerveWheel}
     * @param _rlWheel              The rear left {@code SwerveWheel}
     * @param _rrWheel              The rear right {@code SwerveWheel}
     * @param _maxWheelSpeed        The maximum directional speed that a {@code SwerveWheel} can be set to
     * @param _turnSpeed            A value deciding how quickly the robot will turn, relative to {@code
     * _directMoveSpeed}. This should be adjusted over time while testing. A recommended initial value is
     * 1/2 of {@code _maxWheelSpeed}.
     * @param _directMoveSpeed      A value deciding how quickly the robot will strafe, relative to {@code
     * _turnSpeed}. This should be adjusted over time while testing. A recommended initial value is
     * 1/2 of {@code _maxWheelSpeed}.
     * @param _widthToHeightRatio   The distance between the centers of the front (or back) two wheels divided
     * by the distance between the centers of the right (or left) two wheels. The ratio from the width of the
     * wheelbase to the height of the wheelbase (where "up" is forward). {@link #SwerveDrive(SwerveWheel,
     * SwerveWheel, SwerveWheel, SwerveWheel, double, double, double)} is recommended if this ratio is 1:1.
     */
    public SwerveDrive (
        SwerveWheel _flWheel,
        SwerveWheel _frWheel,
        SwerveWheel _rlWheel,
        SwerveWheel _rrWheel,
        double _maxWheelSpeed,
        double _turnSpeed,
        double _directMoveSpeed,
        double _widthToHeightRatio) {
        
        flWheel = _flWheel;
        frWheel = _frWheel;
        rlWheel = _rlWheel;
        rrWheel = _rrWheel;
        
        maxWheelSpeed = _maxWheelSpeed;
        turnSpeed = _turnSpeed;
        directMoveSpeed = _directMoveSpeed;
        
        widthToHeightRatio = _widthToHeightRatio;
    }
    
    /**
     * Drives the {@code SwerveDrive} given direct movement inputs and rotational inputs,
     * all on the interval [-1, 1], where +y is forwards and +x is to the right.
     * @param directMoveX   The strafing speed in the x direction
     * @param directMoveY   The strafing speed in the y direction
     * @param rotate        The rotational speed, where a positive value rotates clockwise
     * and a negative value rotates counterclockwise
     */
    public void drive (double directMoveX, double directMoveY, double rotate) {
        
        final Vector baseVector = new Vector(directMoveX * directMoveSpeed, directMoveY * directMoveSpeed);
        // Rotate vector FR is the rotation vector that will be added to the FR wheel
        final Vector rotateVectorFR = new Vector(rotate * turnSpeed * widthToHeightRatio, -rotate * turnSpeed);
        
        /*
        Clockwise rotation vector additions.
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
        
        final Vector frVector = baseVector.add(rotateVectorFR);
        final Vector rrVector = baseVector.add(rotateVectorFR.reflectAcrossY());
        final Vector rlVector = baseVector.add(rotateVectorFR.scale(-1));
        final Vector flVector = baseVector.add(rotateVectorFR.reflectAcrossX());
        
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
        
        if (maxSpeed > maxWheelSpeed) {
            flSpeed /= maxSpeed;
            frSpeed /= maxSpeed;
            rlSpeed /= maxSpeed;
            rrSpeed /= maxSpeed;
            
            flSpeed *= maxWheelSpeed;
            frSpeed *= maxWheelSpeed;
            rlSpeed *= maxWheelSpeed;
            rrSpeed *= maxWheelSpeed;
        }
        
        // Sets the final wheel speeds and rotations
        flWheel.setSpeedAndRotation(flVector.getRotationDegrees(), flSpeed);
        frWheel.setSpeedAndRotation(frVector.getRotationDegrees(), frSpeed);
        rlWheel.setSpeedAndRotation(rlVector.getRotationDegrees(), rlSpeed);
        rrWheel.setSpeedAndRotation(rrVector.getRotationDegrees(), rrSpeed);
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
    }
    
    @Override
    public String getDescription () {
        return "swerve drive";
    }
    
}