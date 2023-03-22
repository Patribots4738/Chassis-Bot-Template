// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import hardware.Swerve;


public class Robot extends TimedRobot {
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    private final Swerve swerve = new Swerve(driver, operator);

    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        // swap the left Y and left X values because the field is rotated 90 degrees to the driver station (this may change depending on where the drive is.)
        swerve.periodic();
        swerve.drive(driver.getLeftY(), driver.getLeftX(), driver.getRightX(), true, driver.getAButton());
    }

    @Override
    public void testInit() {}
    @Override
    public void testPeriodic() {}
    @Override
    public void disabledInit() {}
    @Override
    public void disabledPeriodic() {}
}
