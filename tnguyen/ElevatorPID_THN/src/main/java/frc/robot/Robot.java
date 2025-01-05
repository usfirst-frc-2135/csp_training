// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot
{
  // Constants
  private final static double               kDt               = 0.02;  // Loop delay time for simulation
  private final static int                  kGamepadPort      = 0;     // XBox controller USB port
  private final static int                  kMotorCANId       = 5;     // Motor CAN ID assignment
  private final static double               kP                = 0.38;  // PID - proportional value
  private final static double               kI                = 0.0;   // PID - integral value
  private final static double               kD                = 0.0;   // PID - derivative value
  private final static double               kEncoderCPR       = 4096;  // Encoder CPR for CTRE Mag encoder connected to Talon SRX
  private final static double               kConstantOutput   = 0.3;   // Constant percent output value
  private final static double               kMaxVelocity      = 1.0;   // Trapezoidal profile max velocity
  private final static double               kMaxAcceleration  = 1.0;   // Trapezoidal profile max acceleration
  private final static double               kForwardGoal      = 3.0;   // Trapezoidal move - forward goal rotations
  private final static double               kReverseGoal      = 0.0;   // Trapezoidal move - reverse goal rotations
  private final static double               kMaxPositionError = 0.06; //Maximum allowed error between actual position and goal

  // Class member objects
  private final XboxController              m_controller      = new XboxController(kGamepadPort);
  private final ExampleSmartMotorController m_motor           = new ExampleSmartMotorController(kMotorCANId, kEncoderCPR);
  private final TrapezoidProfile            m_profile         =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration));
  private TrapezoidProfile.State            m_goal            = new TrapezoidProfile.State( );
  private TrapezoidProfile.State            m_setpoint        = new TrapezoidProfile.State( );

  private final TalonSRXSimCollection       m_motorSim        = m_motor.getMotorSimulation( );
  private final ElevSim                     m_elevSim         = new ElevSim(m_motorSim, kEncoderCPR);

  private double                            goal;
  private boolean                           m_pidEnabled      = false;

  @Override
  public void robotInit( )
  {
    DataLogManager.start( );
    m_motor.setPID(kP, kI, kD);
    m_motor.resetEncoder( );
    m_elevSim.periodic( );

    SmartDashboard.putNumber("Kp", m_motor.getKp( ));
  }

  @Override
  public void teleopPeriodic( )
  {
    SmartDashboard.putNumber("Elevator Rotations", m_motor.getEncoderDistance( ));
    SmartDashboard.putNumber("Target", goal);
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError( ));
    SmartDashboard.putNumber("Velocity", m_motor.getVelocity( ));

    m_elevSim.periodic( );

    if (m_controller.getAButtonPressed( ))
    {
      m_pidEnabled = false;
      DataLogManager.log("A Button Pressed -- Set motor to constant forward speed (PID disabled)");
      m_motor.set(kConstantOutput);
    }

    if (m_controller.getBButtonPressed( ))
    {
      m_pidEnabled = false;
      DataLogManager.log("B Button Pressed -- Set motor to constant reverse speed (PID disabled)");
      m_motor.set(-kConstantOutput);
    }

    if (m_controller.getXButtonPressed( ))
    {
      m_pidEnabled = true;
      DataLogManager.log("X Button Pressed -- Trapezoid Profile Forward Goal (PID enabled)");
      m_goal = new TrapezoidProfile.State(kForwardGoal, 0);
    }

    if (m_controller.getYButtonPressed( ))
    {
      m_pidEnabled = true;
      DataLogManager.log("Y Button Pressed -- Trapezoid Profile Reverse Goal (PID enabled)");
      m_goal = new TrapezoidProfile.State(kReverseGoal, 0);
    }

    if (m_controller.getRightBumperPressed( ))
    {
      m_pidEnabled = false;
      DataLogManager.log("Right Bumper Pressed -- Stop Motor (PID disabled)");
      m_motor.stopMotor( );
    }

    if (m_controller.getStartButtonPressed( ))
    { // if menu button, invert motor direction
      m_pidEnabled = false;
      DataLogManager.log("Menu Button Pressed - Invert Motor Direction (PID disabled)");
      m_motor.setInverted(!m_motor.getInverted( ));
    }

    if (m_pidEnabled && (Math.abs(m_goal.position - m_setpoint.position) > kMaxPositionError))
    {
      m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, 0.0);
    }
  }
}
