package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Climber extends SubsystemBase {
    private static SUB_Climber INSTANCE = null;
    public static SUB_Climber getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Climber();
        }
    
        return INSTANCE;
    }

    private SUB_Climber (){

    }
}
