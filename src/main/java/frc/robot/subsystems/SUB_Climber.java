package frc.robot.subsystems;

public class SUB_Climber {
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
