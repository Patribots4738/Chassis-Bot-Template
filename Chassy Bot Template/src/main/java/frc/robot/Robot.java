// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import calc.Constants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import hardware.Swerve;


public class Robot extends TimedRobot
{
    // TODO: remove test code
    private final XboxController controller = new XboxController(0);
    private final Swerve swerve = new Swerve();
    
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(3);
    
    
    @Override
    public void autonomousPeriodic()
    {
        driveWithJoystick(false);
        swerve.calibrateOdometry();
    }
    
    @Override
    public void teleopPeriodic()
    {
        driveWithJoystick(true);
    }

    // TODO: remove test code and add calculation to drive method
    private void driveWithJoystick(boolean fieldRelative)
    {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -xSpeedLimiter.calculate(controller.getLeftY()) * Constants.DriveConstants.MAX_SPEED;
        
        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -ySpeedLimiter.calculate(controller.getLeftX()) * Constants.DriveConstants.MAX_SPEED;
        
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -rotateLimiter.calculate(controller.getRightX()) * Constants.DriveConstants.MAX_ANGULAR_SPEED;
        
        swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
}
