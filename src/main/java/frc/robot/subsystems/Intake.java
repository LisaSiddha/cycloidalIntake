package frc.robot.subsystems;
 
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
 
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  //pivot degs
  private static final double STOWED_DEGS = 60;
  private static final double DEPLOYED_DEGS = 120;
 
  //canids
  private static final int PIVOT_LEFT_ID = 9;
  //private static final int PIVOT_RIGHT_ID = 32;
  private static final int ROLLER_LEFT_ID = 11;
  private static final int ROLLER_RIGHT_ID = 10;
 
  
 
  private final TalonFX PIVOT_LEFT = new TalonFX(PIVOT_LEFT_ID);
  //private final TalonFX PIVOT_RIGHT = new TalonFX(PIVOT_RIGHT_ID);
  private final TalonFX ROLLER_LEFT = new TalonFX(ROLLER_LEFT_ID);
  private final TalonFX ROLLER_RIGHT = new TalonFX(ROLLER_RIGHT_ID);
 
  private final MotionMagicDutyCycle pivotRequest = new MotionMagicDutyCycle(0);
  private final VoltageOut rollerRequest = new VoltageOut(0);
 
  public Intake() {
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEARING; 
    pivotConfig.Slot0.kP = 4.0; // tODO: tune this
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    pivotConfig.MotionMagic.MotionMagicAcceleration = 3.0;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(DEPLOYED_DEGS + 5);
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(STOWED_DEGS - 5);
 
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PIVOT_LEFT.getConfigurator().apply(pivotConfig);
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // mirror
    //PIVOT_RIGHT.getConfigurator().apply(pivotConfig);
 
   
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 40;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
 
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ROLLER_LEFT.getConfigurator().apply(rollerConfig);
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    ROLLER_RIGHT.getConfigurator().apply(rollerConfig);
 
   
    PIVOT_LEFT.setPosition(Units.degreesToRotations(STOWED_DEGS));
    //PIVOT_RIGHT.setPosition(Units.degreesToRotations(STOWED_DEGS));
  }
 
  private void setPivot(double degrees) {
    double rotations = Units.degreesToRotations(degrees);
    PIVOT_LEFT.setControl(pivotRequest.withPosition(rotations));
    //PIVOT_RIGHT.setControl(pivotRequest.withPosition(rotations));
  }
 
  private void setRollers(double volts) {
    ROLLER_LEFT.setControl(rollerRequest.withOutput(volts));
    ROLLER_RIGHT.setControl(rollerRequest.withOutput(volts));
  }
 
  
  public Command intake() {
    return startEnd(
        () -> {
          setPivot(DEPLOYED_DEGS);
          setRollers(IntakeConstants.ROLLER_VOLTS);
        },
        () -> {
          setRollers(0);
          setPivot(STOWED_DEGS);
        });
  }
 

  public Command outtake() {
    return startEnd(() -> setRollers(-IntakeConstants.ROLLER_VOLTS), () -> setRollers(0));
  }
 
  
  public Command deploy() {
    return runOnce(() -> setPivot(DEPLOYED_DEGS));
  }
 
  
  public Command stow() {
    return runOnce(() -> setPivot(STOWED_DEGS));
  }
}
 