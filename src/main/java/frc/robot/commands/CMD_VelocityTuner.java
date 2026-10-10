package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Motor.TuneableMotor;
import static edu.wpi.first.units.Units.RPM;

public class CMD_VelocityTuner extends Command {
    private final TuneableMotor motor;
    private final Timer timer = new Timer();
    
    private enum TuneState {
        FIND_KS,
        SETUP_KV_TEST,
        FIND_KV,
        SETUP_KP_TEST,
        RUN_KP_STEP,
        DONE
    }
    
    private TuneState currentState = TuneState.FIND_KS;

    public double kS = 0;
    public double kV = 0;
    public double kA = 0;
    public double kP = 0.0;
    public double recoveryTime = 0.080; 

    private double currentVoltage = 0.0;
    private double sumX, sumY, sumXY, sumXX; private int n;   
    private double kpTestTargetSpeed = 0.0;
    private double t10 = -1.0;
    private double t90 = -1.0;
    private double v10 = 0.0;
    private double v90 = 0.0;

    private double skibidi = 2.0;

    public CMD_VelocityTuner(TuneableMotor motor, double recoveryTime) {
        this.motor = motor;
        this.recoveryTime = recoveryTime/1000.0;
    }

    public CMD_VelocityTuner(TuneableMotor motor) {
        this(motor, 0.080);
    }

    @Override
    public void initialize() {
        currentState = TuneState.FIND_KS;
        currentVoltage = 0.0;
        kP = 0.0;
        kA = 0.0;
        sumX = 0;
        sumY = 0;
        sumXY = 0;
        sumXX = 0;
        n = 0; 
        skibidi = 2.0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        switch (currentState) {
            
            case FIND_KS:
                currentVoltage += 0.01; 
                motor.setVoltage(currentVoltage);
                
                if (motor.getVelocity().in(RPM) > 0.1) {
                    kS = currentVoltage-0.02;
                    motor.setVoltage(0);
                    timer.reset();
                    currentState = TuneState.FIND_KV;
                }
                
                if (currentVoltage >= 6.0) {
                    motor.setVoltage(0);
                    currentState = TuneState.DONE;
                }
                break;
            

            case SETUP_KV_TEST: 
                motor.setVoltage(0);
                skibidi = 2.0;
                currentState = TuneState.FIND_KV;
                break;

            
            case FIND_KV:
                if (skibidi%2==0) {
                    timer.reset();
                    timer.start();
                    motor.setVoltage(kS + (skibidi/2.0) * ((12.0-kS) / 10.0));
                    skibidi++;
                }

                if (timer.get() > 2.0) {
                    double velocity = motor.getVelocity().in(RPM);
                    double targetVolts = ((skibidi-1)/2.0) * ((12.0-kS) / 10.0);
                    if (velocity > 0.5) {
                        sumX += velocity; sumY += targetVolts;
                        sumXY += velocity * targetVolts; sumXX += velocity * velocity; n++;
                    }
                    skibidi++;


                }

                if (skibidi > 21.0) {
                    kV = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
                    motor.setVoltage(0);
                    currentState = TuneState.SETUP_KP_TEST;
                }
                break;

            case SETUP_KP_TEST: 
                motor.setVoltage(0);
                kpTestTargetSpeed = (6.0 - kS) / kV;
                timer.reset();
                t10 = -1.0;
                t90 = -1.0;
                v10 = 0.0;
                v90 = 0.0;
                if (motor.getVelocity().in(RPM) > 5.0) {
                    currentState = TuneState.SETUP_KP_TEST; 
                    currentState = TuneState.RUN_KP_STEP;
                }
                break;

            case RUN_KP_STEP:
                motor.setVoltage(6.0);
                double currentSpeed = motor.getVelocity().in(RPM);
                
                if (t10 < 0 && currentSpeed >= 0.20 * kpTestTargetSpeed) {
                    t10 = timer.get();
                    v10 = currentSpeed;
                }
                if (t90 < 0 && currentSpeed >= 0.70 * kpTestTargetSpeed) {
                    t90 = timer.get();
                    v90 = currentSpeed;
                }
                
                if ((t90 > 0 && t10 > 0) || timer.get() >= 1.5) {
                    if (t90 > 0 && t10 > 0 && t90 > t10) {
                        double dt = t90 - t10;
                        double dw = v90 - v10;
                        double alpha = dw / dt;
                        double avgSpeed = (v10 + v90) / 2.0;
                        double netVolts = 6.0 - kS - (kV * avgSpeed);
                        kA = netVolts / alpha;
                        kP = kA / recoveryTime;
                    }
                    motor.setVoltage(0);
                    currentState = TuneState.DONE;
                }
                break;

            case DONE:
                motor.setVoltage(0);
                break;
        }


        SmartDashboard.putNumber("VelocityTuner/State", currentState.ordinal());
        SmartDashboard.putNumber("VelocityTuner/S", kS);
        SmartDashboard.putNumber("VelocityTuner/V", kV);
        SmartDashboard.putNumber("VelocityTuner/A", kA);
        SmartDashboard.putNumber("VelocityTuner/P", kP);

        SmartDashboard.putNumber("VelocityTuner/Current Voltage", currentVoltage);
        SmartDashboard.putNumber("VelocityTuner/kpTestTargetSpeed", kpTestTargetSpeed);
        SmartDashboard.putNumber("VelocityTuner/skibidi", skibidi);
    }

    @Override
    public boolean isFinished() {
        return currentState == TuneState.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        motor.setVoltage(0);
        timer.stop();
        
        System.out.println("--- TUNING RESULTS ---");
        System.out.println("Calculated kS: " + kS);
        System.out.println("Calculated kV: " + kV);
        System.out.println("Calculated kA: " + kA);
        System.out.println("Optimized kP:  " + kP + " (for " + String.format("%.0f ms", recoveryTime * 1000.0) + " recovery)");
    }

    
}
