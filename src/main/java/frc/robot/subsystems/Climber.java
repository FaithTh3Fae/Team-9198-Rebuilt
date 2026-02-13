package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{

    private final SparkMax climbMotor;

    public Climber() {
        climbMotor = new SparkMax(11, MotorType.kBrushed);
        SparkMaxConfig climbConfig = new SparkMaxConfig();
        climbConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
                climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void periodic() {

    }

    public void climbStop(){
        climbMotor.set(0);
    }

    public void runClimbMotor(double speed) {
        climbMotor.set(speed);
    }
    
    public Command PrepareClimber(double speed){
        return this.startEnd(()->runClimbMotor(speed),()->runClimbMotor(0));
    }
}
