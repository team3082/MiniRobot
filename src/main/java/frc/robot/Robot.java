// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Subsystems.*;
import frc.robot.Utilities.*;


public class Robot extends TimedRobot {


  @Override
  public void robotInit() {
    Pigeon.init();
    Pigeon.zero();
    SwerveManager.init();
    RTime.init();
    OI.init();
  }

  @Override
  public void robotPeriodic() {
    Pigeon.update();
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    OI.init();
  }

  @Override
  public void teleopPeriodic() {
    RTime.update();
    OI.joystickInput();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
