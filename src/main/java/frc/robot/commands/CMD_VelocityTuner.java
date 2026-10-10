package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Motor.TuneableMotor;
import static edu.wpi.first.units.Units.RPM;

public class CMD_VelocityTuner extends Command {
    private final TuneableMotor motor;
    private final Timer timer = new Timer();
    
    // States for our tuning sequence
    private enum TuneState {
        FIND_KS,
        SETUP_KV_TEST,
        FIND_KV,
        SETUP_KP_TEST,
        RUN_KP_STEP,
        DONE
    }
    
    private TuneState currentState = TuneState.FIND_KS;

    // Results
    public double kS = 0;
    public double kV = 0;
    public double kP = 0.0001; // Starting guess: 1V per 100 RPM error

    // Internal tracking variables
    private double currentVoltage = 0.0;
    private double sumX, sumY, sumXY, sumXX; private int n;   
    // kP Optimization variables
    private double kpTestTargetSpeed = 0.0;
    private double maxOvershoot = 0.0;
    private double riseTime = 0.0;
    private boolean crossedTarget = false;

    private double skibidi = 2.0;

    public CMD_VelocityTuner(TuneableMotor motor) {
        this.motor = motor;
    }

    @Override
    public void initialize() {
        currentState = TuneState.FIND_KS;
        currentVoltage = 0.0;
        kP = 0.0001; 
        sumX=0;
        sumY=0;
        sumXY=0;
        sumXX=0;
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
                    kS = currentVoltage-0.02; // Account for 0.05 step size and for that the velocity doesn't instantly update
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

            case SETUP_KP_TEST: // This is the most confusing one for me and for whoever reads this, so more comments are here
                motor.setVoltage(0); // Stop the motor before starting the kP test
                kpTestTargetSpeed = (6.0 - kS) / kV; // 6V - kS, divided by kV to get the speed the motor runs at 6V
                timer.reset(); // Reset the timer
                crossedTarget = false; // This tracks whether in 1.5 seconds we crossed the target speed or not
                maxOvershoot = -kpTestTargetSpeed; // This tracks the maximum overshoot we had in 1.5 seconds
                if (motor.getVelocity().in(RPM) > 0.1) {
                    currentState = TuneState.SETUP_KP_TEST; // If the motor is still moving, we wait until it stops before starting the kP test
                } else {
                    currentState = TuneState.RUN_KP_STEP; // Start the kP test! I hope this isn't super violent not trying to blow up another motor lol
                }
                break;

            case RUN_KP_STEP:
                double currentSpeed = motor.getVelocity().in(RPM); // Get the current speed of the motor
                double error = kpTestTargetSpeed - currentSpeed; // Calculate the error between the target speed and the current speed
                motor.setVoltage(kS + (kV * kpTestTargetSpeed * 0.90) + (kP * error)); // Do SVAPID
                
                if (!crossedTarget && currentSpeed >= kpTestTargetSpeed) { // If we just crossed the target speed, we record the rise time and set crossedTarget to true
                    riseTime = timer.get();
                    crossedTarget = true;
                }
                
                double overshoot = currentSpeed - kpTestTargetSpeed;
                if (overshoot > maxOvershoot && timer.get()>0.5) {
                    maxOvershoot = overshoot;
                }
                
                // Once we reach 1.5 seconds, no matter what happens we evaluate
                if (timer.get() >= 1.5) {
                    double percentOff = (maxOvershoot / kpTestTargetSpeed) * 100.0; // Calculate the percent overshoot
                    
                    if (percentOff <= 5.0 && percentOff >= 0.0) { 
                        currentState = TuneState.DONE;
                    } else { 
                        kP -= 2*kP*percentOff/100.0;
                        currentState = TuneState.SETUP_KP_TEST;
                    }

                }
                break;

            case DONE:
                motor.setVoltage(0);
                break;
        }


        SmartDashboard.putNumber("VelocityTuner/State", currentState.ordinal());
        SmartDashboard.putNumber("VelocityTuner/S", kS);
        SmartDashboard.putNumber("VelocityTuner/V", kV);
        SmartDashboard.putNumber("VelocityTuner/P", kP);

        SmartDashboard.putNumber("VelocityTuner/Current Voltage", currentVoltage);
        
        SmartDashboard.putNumber("VelocityTuner/kpTestTargetSpeed", kpTestTargetSpeed);
        SmartDashboard.putNumber("VelocityTuner/Max Overshoot",maxOvershoot);
        SmartDashboard.putNumber("VelocityTuner/riseTime", riseTime);
        SmartDashboard.putBoolean("VelocityTuner/crossedTarget", crossedTarget);
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
        System.out.println("Optimized kP:  " + kP);
    }

    
}
