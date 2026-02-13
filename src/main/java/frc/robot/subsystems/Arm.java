package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{

    private final SparkMax armMotor;

    public Arm() {
        armMotor = new SparkMax(10, MotorType.kBrushed);
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
                armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void periodic() {

    }

    public void runArmMotor(double speed) {
        armMotor.set(speed);
    }
    
    public Command PrepareArm(double speed){
        return this.startEnd(()->runArmMotor(speed),()->runArmMotor(0));
    }
}
