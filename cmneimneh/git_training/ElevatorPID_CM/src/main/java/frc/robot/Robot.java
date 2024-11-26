// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot
{
  private static double                     kDt           = 0.02;
  private double                            goal;
  private final static double               kEncoderCPR   = 4096;

  private final XboxController              m_controller  = new XboxController(0);
  private final ExampleSmartMotorController m_motor       = new ExampleSmartMotorController(5, kEncoderCPR);
  private final SimpleMotorFeedforward      m_feedforward = new SimpleMotorFeedforward(1, 1.5);
  private final TrapezoidProfile            m_profile     = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.0, 1.0));
  private TrapezoidProfile.State            m_goal        = new TrapezoidProfile.State( );
  private TrapezoidProfile.State            m_setpoint    = new TrapezoidProfile.State( );

  private final TalonSRXSimCollection       m_motorSim    = m_motor.getMotorSimulation( );
  private final ElevSim                     m_elevSim     = new ElevSim(m_motorSim, kEncoderCPR);
  private boolean                           m_pidEnabled  = false;

  @Override
  public void robotInit( )
  {
    m_motor.setPID(0.38, 0.0, 0.0);
    m_motor.resetEncoder( );
    m_elevSim.periodic( );
    DataLogManager.start( );
  }

  @Override
  public void teleopPeriodic( )
  {
    SmartDashboard.putNumber("Elevator Rotations", m_motor.getEncoderDistance( ));
    SmartDashboard.putNumber("Target", goal);
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError( ));
    SmartDashboard.putNumber("Kp", m_motor.getKp( ));
    SmartDashboard.putNumber("Velocity", m_motor.getVelocity( ));

    m_elevSim.periodic( );

    if (m_controller.getRightBumperPressed( ))
    { // if Right Bumper pressed, stop Motor
      m_pidEnabled = false;
      DataLogManager.log("PID Disabled");
      m_motor.stopMotor( );
      DataLogManager.log("Right Bumper Pressed -- Motor Stopped");
    }

    if (m_controller.getRawButtonPressed(8))
    { // if menu button, invert motor direction
      DataLogManager.log("Menu Button Pressed");
      m_pidEnabled = false;
      DataLogManager.log("PID Disabled");

      m_motor.setInverted(!m_motor.getInverted( ));
      DataLogManager.log("Motor Inverted -- " + m_motor.getInverted( ));
      DataLogManager.log(String.format("Motor Inverted -- %s", m_motor.getInverted( ) ? "true" : "false"));
      DataLogManager.log(String.format("Motor Output -- %.1f", m_motor.get( )));
    }

    if (m_controller.getAButtonPressed( ))
    { // if A button pressed, set voltage of 0.3
      m_pidEnabled = false;
      DataLogManager.log("PID Disabled");
      m_motor.set(0.3);
      DataLogManager.log("A Button Pressed -- Voltage PercentOutput: 0.3");
    }

    if (m_controller.getBButtonPressed( ))
    { // if B button pressed, set voltage of -0.3
      m_pidEnabled = false;
      DataLogManager.log("PID Disabled");
      m_motor.set(-0.3);
      DataLogManager.log("B Button Pressed -- Voltage PercentOutput: -0.3");
    }

    if (m_controller.getXButtonPressed( ))
    {
      m_pidEnabled = true;
      DataLogManager.log("PID Enabled");
      m_goal = new TrapezoidProfile.State(3, 0);
      DataLogManager.log("X Button Pressed -- Trapezoid Profile Setpoint: 1.0");
    }

    if (m_controller.getYButtonPressed( ))
    {
      m_pidEnabled = true;
      DataLogManager.log("PID Enabled");
      m_goal = new TrapezoidProfile.State(0, 0);
      DataLogManager.log("Y Button Pressed -- Trapezoid Profile Setpoint: 0.0");
    }

    if (m_pidEnabled)
    {
      m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, 0.0);
    }
  }
}

// take out repetitive PID logs
// Move PID loops to bottom of periodic
