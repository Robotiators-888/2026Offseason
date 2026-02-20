// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class SUB_Climber extends SubsystemBase {
//     private SparkMax climberMotor;

//     private static SUB_Climber INSTANCE = null;
//     public static SUB_Climber getInstance (){
//         if (INSTANCE == null) {
//             INSTANCE = new SUB_Climber();
//         }
    
//         return INSTANCE;
//     }

//     private SUB_Climber () {
//         climberMotor = new SparkMax(Constants.Climber.kCLIMBER_MOTOR_CANID, SparkMax.MotorType.kBrushless);
//         SparkMaxConfig config = new SparkMaxConfig();
//         config.encoder.positionConversionFactor(1); // Rotations
//         config.encoder.velocityConversionFactor(1 / 60.0); // Converts RPM to rot/sec
//         config.smartCurrentLimit(35);
//         config.inverted(true);
//         // climberMotor.getEncoder().setPosition(0);
//         climberMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

//     }

//     public void setClimber(double speed){
//         climberMotor.set(speed);
//     }

//     public boolean hasReachedSetPoint (boolean isExtending) {
//         return (isExtending) ? Math.abs(climberMotor.getEncoder().getPosition() - Constants.Climber.kCLIMBER_SETPOINT)<Constants.Climber.kCLIMBER_TOLERANCE : climberMotor.getEncoder().getPosition()<Constants.Climber.kCLIMBER_TOLERANCE;
//     }


//     public void periodic() {
//         SmartDashboard.putNumber("ClimberArmPositionInches", climberMotor.getEncoder().getPosition()*Constants.Climber.kCLIMBER_CONVERSION);
//     }


//     public void climb() {
//         setClimber(Constants.Climber.kCLIMBER_MOTOR_SPEED); // Yeah its backwards for some reason
//     }

//     public void unClimb() {
//         setClimber(-Constants.Climber.kCLIMBER_MOTOR_SPEED);
//     }

//     public void stopClimb() {
//         setClimber(0);
//     }
// }
