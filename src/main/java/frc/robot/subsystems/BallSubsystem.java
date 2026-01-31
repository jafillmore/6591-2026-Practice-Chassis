// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.BallConstants;

public class BallSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  private final SparkMax m_intakeSpark = new SparkMax(BallConstants.kintakeCANId, MotorType.kBrushed);
  private final SparkMax m_shooterSpark = new SparkMax(BallConstants.kshooterCANId, MotorType.kBrushed);
    


  public BallSubsystem() {

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_intakeSpark.configure(Configs.Coral.coralConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void activateBalls (double shootPower, double intakePower) {
    m_shooterSpark.set(shootPower);
    m_intakeSpark.set(intakePower);

  };



  



}
