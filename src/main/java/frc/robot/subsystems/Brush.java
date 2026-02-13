// It might be simpler to run all brushed motors through a single subsystem
// Pass in the motor ID to the constructor
// Example: new Brush(10) for motor ID 10
// Only do this if the code is the same
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Brush extends SubsystemBase{

    private final SparkMax brushMotor;

    public Brush(int motorID) {
        brushMotor = new SparkMax(motorID, MotorType.kBrushed);
        SparkMaxConfig brushConfig = new SparkMaxConfig();
        brushConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
                brushMotor.configure(brushConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void periodic() {

    }

    public void runBrushMotor(double speed) {
        brushMotor.set(speed);
    }

    public void brushStop(){
        brushMotor.set(0);
    }
    
    public Command prepareBrush(double speed){
        return this.startEnd(()->runBrushMotor(speed),()->runBrushMotor(0));
    }
}
