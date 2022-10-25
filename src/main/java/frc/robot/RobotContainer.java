// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Module1;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }



  private static RobotContainer me;
  private final Module1 module;
  private static Joystick joystickxy;
  private static Joystick joystickDirection;
  private static PigeonIMU jyro;

  private RobotContainer() {
    me = this;
    module = new Module1();
    joystickDirection = new Joystick(Constants.Buttons.joystickDirections_PORT_NUM);
    joystickxy = new Joystick(Constants.Buttons.joystickxy_PORT_NUM);
    jyro = new PigeonIMU(Constants.ChassiConst.jyro_PORT_NUM);
  }

  public RobotContainer getRobotContainer() {
    configureButtonBindings();
    if (me==null) {
      return new RobotContainer();
    }
    return me;
  }

  public static PigeonIMU getJyro() {
    return jyro;
  }

  public static Joystick getJoystickXY() {
    return joystickxy;
  }

  public static Joystick getJoystickDirection() {
    return joystickDirection;
  }
  

}
