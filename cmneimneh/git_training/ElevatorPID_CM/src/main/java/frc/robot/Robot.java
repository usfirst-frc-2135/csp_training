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
  // Constants
  // TODO: Note that we try to make all numbers use names (these are called literals) to make them describe the value
  //    Use these literals to replace the "magic" numbers in your code--it should improve readability
  private final static double               kDt              = 0.02;  // Loop delay time for simulation
  private final static int                  kGamepadPort     = 0;     // XBox controller USB port
  private final static int                  kMotorCANId      = 5;     // Motor CAN ID assignment
  private final static double               kP               = 0.38;  // PID - proportional value
  private final static double               kI               = 0.0;   // PID - integral value
  private final static double               kD               = 0.0;   // PID - derivative value
  private final static double               kEncoderCPR      = 4096;  // Encoder CPR for CTRE Mag encoder connected to Talon SRX
  private final static double               kConstantOutput  = 0.3;   // Constant percent output value
  private final static double               kMaxVelocity     = 1.0;   // Trapezoidal profile max velocity
  private final static double               kMaxAcceleration = 1.0;   // Trapezoidal profile max acceleration
  private final static double               kForwardGoal     = 3.0;   // Trapezoidal move - forward goal rotations
  private final static double               kReverseGoal     = 0.0;   // Trapezoidal move - reverse goal rotations

  // Class member objects
  private final XboxController              m_controller     = new XboxController(0);
  private final ExampleSmartMotorController m_motor          = new ExampleSmartMotorController(5, kEncoderCPR);
  private final SimpleMotorFeedforward      m_feedforward    = new SimpleMotorFeedforward(1, 1.5);
  private final TrapezoidProfile            m_profile        = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.0, 1.0));
  private TrapezoidProfile.State            m_goal           = new TrapezoidProfile.State( );
  private TrapezoidProfile.State            m_setpoint       = new TrapezoidProfile.State( );

  private final TalonSRXSimCollection       m_motorSim       = m_motor.getMotorSimulation( );
  private final ElevSim                     m_elevSim        = new ElevSim(m_motorSim, kEncoderCPR);
  private boolean                           m_pidEnabled     = false;
  private double                            goal;   // TODO: I think you'll find that this variable is never used-- m_goal should be used on the dashboard

  @Override
  public void robotInit( )
  {
    DataLogManager.start( );  // TODO: This should be first, so the methods that follow will get logged!
    m_motor.setPID(0.38, 0.0, 0.0);
    m_motor.resetEncoder( );
    m_elevSim.periodic( );
  }

  @Override
  public void teleopPeriodic( )
  {
    SmartDashboard.putNumber("Elevator Rotations", m_motor.getEncoderDistance( ));
    SmartDashboard.putNumber("Target", goal); // TODO: This should be m_goal
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError( ));
    SmartDashboard.putNumber("Kp", m_motor.getKp( )); // TODO: Since this value is not changed while the robot is running it should go in robotInit
    SmartDashboard.putNumber("Velocity", m_motor.getVelocity( ));

    m_elevSim.periodic( );

    if (m_controller.getRightBumperPressed( ))
    { // if Right Bumper pressed, stop Motor // TODO: This comment now just repeats what the code is saying--delete it (and the other examples below).
      m_pidEnabled = false;
      // DataLogManager.log("PID Disabled"); // TODO: Put this into your log message that is already being printed. I did some of them.
      DataLogManager.log("Right Bumper Pressed -- Stop motor (PID disabled)");
      m_motor.stopMotor( );
    }

    if (m_controller.getRawButtonPressed(8))
    { // if menu button, invert motor direction // TODO: redundant comment - delete me
      DataLogManager.log("Menu Button Pressed - Invert motor (PID disabled)");
      m_pidEnabled = false;
      // DataLogManager.log("PID Disabled");

      m_motor.setInverted(!m_motor.getInverted( ));
      // TODO: Looks like you're testing out string concatenation vs. string formatting (that's what I would do, too, to try it). Clean up the spam when you're done and only leave one.
      // TODO: BTW, the setInverted command is going to log a message with the new value, so this log message is probably redundant
      DataLogManager.log("Motor Inverted -- " + m_motor.getInverted( ));
      DataLogManager.log(String.format("Motor Inverted -- %s", m_motor.getInverted( ) ? "true" : "false"));
      DataLogManager.log(String.format("Motor Output -- %.1f", m_motor.get( ))); // TODO: This is the percentOutput value, so it probably doesn't go with the inversion logic
    }

    if (m_controller.getAButtonPressed( ))
    { // if A button pressed, set voltage of 0.3 // TODO: redundant comment - delete me
     // DataLogManager.log("A Button Pressed -- Voltage PercentOutput: 0.3 (PID disabled)");
      DataLogManager.log("A Button Pressed -- Set motor to constant forward speed");
      m_pidEnabled = false;
      // DataLogManager.log("PID Disabled");
      m_motor.set(0.3);
    }

    if (m_controller.getBButtonPressed( ))
    { // if B button pressed, set voltage of -0.3 // TODO: redundant comment - delete me
     // DataLogManager.log("B Button Pressed -- Voltage PercentOutput: -0.3");
      DataLogManager.log("A Button Pressed -- Set motor to constant reverse speed (PID disabled)");
      m_pidEnabled = false;
      // DataLogManager.log("PID Disabled");
      m_motor.set(-0.3);
    }

    if (m_controller.getXButtonPressed( ))
    {
      // DataLogManager.log("X Button Pressed -- Trapezoid Profile Setpoint: 1.0");
      DataLogManager.log("X Button Pressed -- Trapezoid Profile Forward Goal (PID enabled)");
      m_pidEnabled = true;
      DataLogManager.log("PID Enabled");
      m_goal = new TrapezoidProfile.State(3, 0);
    }

    if (m_controller.getYButtonPressed( ))
    {
      // DataLogManager.log("Y Button Pressed -- Trapezoid Profile Setpoint: 0.0");
      DataLogManager.log("Y Button Pressed -- Trapezoid Profile Reverse Goal (PID enabled)");
      m_pidEnabled = true;
      // DataLogManager.log("PID Enabled");
      m_goal = new TrapezoidProfile.State(0, 0);
    }

    if (m_pidEnabled)
    {
      m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, 0.0);
      // TODO: In order to stop the spamming after the move has completed, how can we detect we are at the m_goal?
      //        Won't the current position be equal to (or actually close to) the target goal? At that point, do we
      //        need to call setSetpoint anymore? (...no...)
    }
  }
}

// TODO: I think these are done now -- if so, delete them
// take out repetitive PID logs 
// Move PID loops to bottom of periodic
