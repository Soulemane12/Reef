package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  // Instantiate the motor on PWM/Can ID 10 (adjust if needed) as brushless.
  private final TalonFX motor =new TalonFX(30, "ChooChooTrain");


  /** Sets the shooter motor speed.
   * @param speed The speed to set. Positive values shoot forward.
   */
  public void shoot(double speed) {
    // Reverse the sign if needed to match your motor wiring.
    motor.set(-speed);
  }
  
  @Override
  public void periodic() {
    // Optional: add any periodic shooter updates here.
  }
}
