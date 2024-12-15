// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class Robot extends TimedRobot
{
  private TrapezoidProfile    m_trapezoidalProfile;
  private static final double kEncoderCPR = 4096;

  private static double       kDt         = 0.02;

  // private final static double kv = 1.0;
  // private final static double ka = 2.0;
  public void TrapezoidalProfile( )
  {
    final double kv = 1.0;
    final double ka = 2.0;
    TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);
  }

  private final XboxController                     m_controller  = new XboxController(0);
  private final SimpleMotorFeedforward             m_feedforward = new SimpleMotorFeedforward(1, 1.0);
  private final static ExampleSmartMotorController m_TopMotor    = new ExampleSmartMotorController(5);
  // Note: These gains are fake, and will have to be tuned for your robot.

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile                   m_profile     =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State                   m_goal        = new TrapezoidProfile.State( );
  private TrapezoidProfile.State                   m_setpoint    = new TrapezoidProfile.State( );
  // public final static TalonSRXSimCollection m_TopMotorSim =
  // m_TopMotor.getMotorSimulation();m_motorSim=m_motor.getSimCollection();
  // public final static elevSim m_elevSim = new elevSim(m_motorSim, kEncoderCPR);

  public class elevSim
  {

    public void periodic( )
    {

    }

  }

  @Override
  public void robotInit( )
  {
    m_TopMotor.resetEncoder( );
    // m_elevSim.periodic();
    // Note: These gains are fake, and will have to be tuned for your robot.
    m_TopMotor.setPID(0.125, 0, 0);
  }

  @Override
  public void teleopPeriodic( )
  {

    // m_elevSim.periodic();

    SmartDashboard.putNumber("elevator_rotations", m_TopMotor.getEncoderDistance( ));
    // SmartDashboard.putNumber("Goal", goal);
    // SmartDashboard.putNumber("Error", )

    if (m_controller.getAButtonPressed( ))
    {
      m_TopMotor.set(0.3);
      DataLogManager.log("set a 0.3(percent output) constant speed");
    }
    if (m_controller.getBButtonPressed( ))
    {
      m_TopMotor.set(-0.3);
      DataLogManager.log("set a -0.3(percent output) constant speed");
    }
    if (m_controller.getRawButtonPressed(8))
    {
      if (m_TopMotor.getInverted( ))
      {
        m_TopMotor.setInverted(false);
      }
      else
      {
        m_TopMotor.setInverted(true);
      }
    }
    double goal;
    if (m_controller.getXButtonPressed( ))
    {
      m_TopMotor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 1.0, 0.0);
      goal = 4096.0;

    }
    if (m_controller.getYButtonPressed( ))
    {
      m_TopMotor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 0.0, 0.0);
      goal = 0.0;

    }
  }

}
// DataLogManager.log("Motor is Inverted");
// } else {
// DataLogManager.log("Motor direction is normal");
// }

// }

// Retrieve the profiled setpoint for the next timestep. This setpoint moves
// toward the goal while obeying the constraints.
/*
 * m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
 * 
 * // Send setpoint to offboard controller PID
 * m_TopMotor.setSetpoint(
 * ExampleSmartMotorController.PIDMode.kPosition,
 * m_setpoint.position,
 * m_feedforward.calculate(m_setpoint.velocity) / 12.0);
 * }
 */
