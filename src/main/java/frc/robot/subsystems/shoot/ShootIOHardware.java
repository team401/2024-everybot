package frc.robot.subsystems.shoot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoot extends SubsystemBase {

int leftMotorID = 0;
int rightMotorID = 1;
CANSparkMax leftMotor = new CANSparkMax(leftMotorID,MotorType.kBrushless);
CANSparkMax lightMotor = new CANSparkMax(rightMotorID,MotorType.kBrushless);





}

