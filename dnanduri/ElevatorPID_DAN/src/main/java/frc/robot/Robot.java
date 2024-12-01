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
  private final static double                      kDt              = 0.020; // Loop delay time for simulation
  private final static int                         kGamepadPort     = 0;     // XBox controller USB port
  private final static int                         kMotorCANId      = 5;     // Motor CAN ID assignment
  private final static double                      kP               = 0.38;  // PID - proportional value
  private final static double                      kI               = 0.0;   // PID - integral value
  private final static double                      kD               = 0.0;   // PID - derivative value
  private final static double                      kEncoderCPR      = 4096;  // Encoder CPR for CTRE Mag encoder connected to Talon SRX
  private final static double                      kConstantOutput  = 0.3;   // Constant percent output value
  private final static double                      kMaxVelocity     = 1.0;   // Trapezoidal profile max velocity
  private final static double                      kMaxAcceleration = 1.0;   // Trapezoidal profile max acceleration
  private final static double                      kForwardGoal     = 3.0;   // Trapezoidal move - forward goal rotations
  private final static double                      kReverseGoal     = 0.0;   // Trapezoidal move - reverse goal rotations

  private final static double                      kv               = 1.0; // Max velocity - RPS 
  private final static double                      ka               = 2.0;

  private final XboxController                     controller       = new XboxController(0);
  private final static ExampleSmartMotorController m_motor          = new ExampleSmartMotorController(5, kEncoderCPR); // TODO: This was missing the second parameter

  // Note: These gains are fake, and will have to be tuned for your robot.  // TODO: Comment no longer valid - delete it.
  private final SimpleMotorFeedforward             m_feedforward    = new SimpleMotorFeedforward(1, 1.5); // TODO: Not used - delete this.

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile                   m_profile        =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));

  // private final TrapezoidProfile.Constraints       m_Constraints    = new TrapezoidProfile.Constraints(kv, ka); // TODO: Not used - delete this.

  private TrapezoidProfile.State                   m_setpoint       = new TrapezoidProfile.State( );
  private double                                   goal;

  private final TalonSRXSimCollection              m_motorSim       = m_motor.getMotorSimulation( );
  private final ElevSim                            m_elevSim        = new ElevSim(m_motorSim, kEncoderCPR);

  // TODO: Not sure why this is in here. Let's talk about what you're trying to do.
  //        I put this into it's own file and it compiles
  // public class ElevSim
  // {
  //   private static final double   kGearRatio          = 40.0;
  //   private static final double   kCarriageMassKg     = 2.0;
  //   private static final double   kDrumDiameterMeters = 2.0 / 39.37;  // Drum diameter in meters (make meter = rotation)
  //   private static final double   kLengthMeters       = 10.0;         // Maximum length in meters
  //   private static final double   kDrumCircumMeters   = kDrumDiameterMeters * Math.PI;      // Drum diameter in meters
  //   // private static final double   kRolloutRatioMeters = kDrumCircumMeters / kGearRatio;     // Meters per shaft rotation

  //   private final ElevatorSim     m_elevatorSim       = new ElevatorSim(DCMotor.getVex775Pro(1), kGearRatio, kCarriageMassKg,
  //       kDrumDiameterMeters / 2, -kLengthMeters, kLengthMeters, false, 0.0);

  //   private TalonSRXSimCollection m_motorSim;
  //   private double                m_cpr;

  //   public ElevSim(TalonSRXSimCollection motorSim, double encoderCPR)
  //   {
  //     TalonSRXSimCollection motorSim2 = motorSim;
  //     m_motorSim = motorSim2;
  //     m_cpr = encoderCPR;

  //   }

  //   public void periodic( )
  //   {
  //     m_motorSim.setBusVoltage(RobotController.getInputVoltage( ));
  //     m_elevatorSim.setInput(m_motorSim.getMotorOutputLeadVoltage( ));

  //     m_elevatorSim.update(0.020);

  //     m_motorSim.setQuadratureRawPosition((int) (m_cpr * m_elevatorSim.getPositionMeters( ) / kDrumCircumMeters));
  //     m_motorSim.setQuadratureVelocity((int) (m_cpr * (m_elevatorSim.getVelocityMetersPerSecond( ) / kDrumCircumMeters) / 10));

  //     RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps( )));

  //     SmartDashboard.putNumber("SIM-motorVolts", m_motorSim.getMotorOutputLeadVoltage( ));
  //     SmartDashboard.putNumber("SIM-elevPos", m_elevatorSim.getPositionMeters( ));
  //     SmartDashboard.putNumber("SIM-elevVel", m_elevatorSim.getVelocityMetersPerSecond( ));

  //   }

  //   public void reset( )
  //   {
  //     m_motorSim.setQuadratureRawPosition((int) (m_cpr * m_elevatorSim.getPositionMeters( ) / kDrumCircumMeters));
  //     m_motorSim.setQuadratureVelocity((int) (m_cpr * (m_elevatorSim.getVelocityMetersPerSecond( ) / kDrumCircumMeters) / 10));

  //     m_elevatorSim.setState(0.0, 0.0);
  //     m_elevatorSim.setInput(0.0);
  //   }
  // }

  @Override
  public void robotInit( )
  {
    DataLogManager.start( );  // TODO: Put this first so all the following methods can use the logger.
    // Note: These gains are fake, and will have to be tuned for your robot.
    m_motor.setPID(0.125, 0.0, 0.0);
    m_motor.resetEncoder( );

    // TODO: Since the motor has no "home" position, no need to initialize the encoders
    DataLogManager.log("Initial encoder position: " + m_motor.getEncoderDistance( ));
  }

  @Override
  public void teleopPeriodic( )
  {
    SmartDashboard.putNumber("elevator rotations", m_motor.getEncoderDistance( ));
    SmartDashboard.putNumber("Goal", goal);
    SmartDashboard.putNumber("Kp", m_motor.getKp( ));
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError( ));

    m_elevSim.periodic( );

    if (controller.getAButtonPressed( ))
    {
      DataLogManager.log("A button pressed"); // TODO: Putting these log messages before the calls will cause the output messages make more sense.
      m_motor.set(0.3);
    }
    else if (controller.getBButtonPressed( ))
    {
      DataLogManager.log("B button pressed");
      m_motor.set(-0.3);
    }

    if (controller.getRawButtonPressed(8))
    {
      if (m_motor.getInverted( ))
      {
        DataLogManager.log("Motor Inverted = false");
        m_motor.setInverted(false);
      }
      else
      {
        DataLogManager.log("Motor Inverted = true");
        m_motor.setInverted(true);
      }
    }

    if (controller.getXButtonPressed( ))
    {
      DataLogManager.log("X button pressed, motor stopped");
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, 0.0);
      goal = 4096.0;
    }

    if (controller.getYButtonPressed( ))
    {
      DataLogManager.log("Y button pressed, motor stopped");
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, 0.0);
      goal = 0.0;
    }

    if (controller.getRightBumperPressed( ))
    {
      DataLogManager.log("Right bumper pressed, motor stopped");
      m_motor.stopMotor( );
    }

    // var profile = new TrapezoidProfile(m_Constraints, m_goal, m_setpoint);
    // var profile = new TrapezoidProfile(m_Constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    // m_setpoint = m_profile.calculate(kDt);

    // Send setpoint to offboard controller PID
    // m_motor.setSetpoint(
    // ExampleSmartMotorController.PIDMode.kPosition,
    // m_setpoint.position,
    // m_feedforward.calculate(m_setpoint.velocity) / 12.0);
  }
}
