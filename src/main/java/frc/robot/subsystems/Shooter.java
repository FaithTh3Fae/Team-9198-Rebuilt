package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    private final SparkMax shootMotor;

    public Shooter() {
        shootMotor = new SparkMax(9, MotorType.kBrushed);
        SparkMaxConfig shootConfig = new SparkMaxConfig();
        shootConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        shootMotor.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void periodic() {

    }

    public void runShootMotor(double speed) {
        shootMotor.set(speed);
    }
    
    public Command PrepareShooter(double speed){
        return this.startEnd(()->runShootMotor(speed),()->runShootMotor(0));
    }
}
