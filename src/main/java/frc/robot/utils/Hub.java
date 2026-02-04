package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Hub {
    private static String gameData = "";

    public static Optional<Boolean> isAllianceHubActive () {
        if (getActiveAlliance().isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(getActiveAlliance().get() == DriverStation.getAlliance().orElse(Alliance.Red));
    }

    public static Optional<Alliance> getActiveAlliance () {
        if (DriverStation.isAutonomous()) {
            return Optional.empty();
        }
        
        double matchTime = DriverStation.getMatchTime();
        if (gameData.length() == 0) {
            gameData = DriverStation.getGameSpecificMessage(); 
            if (gameData.length() == 0) {
                return Optional.empty();
            }
        }
        
        Alliance initialAlliance = gameData.charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;

        if (matchTime >= 130 || matchTime < 30) {
            return Optional.of(DriverStation.getAlliance().orElse(Alliance.Red));
        }
        else if (matchTime >= 105 || (matchTime < 80 && matchTime >= 55)) {
            return initialAlliance == Alliance.Red ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
        }
        else {
            return Optional.of(initialAlliance);
        }
    }

    public static double getTimeUntilNextChange () {
        double matchTime = DriverStation.getMatchTime();
        // Auto and trasition shift
        if (matchTime <= 30) {
            return 30 - matchTime;
        }
        // Shift 1
        else if (matchTime <= 30+(25*1)) {
            return 30+(25*1) - matchTime;
        }
        // Shift 2
        else if (matchTime <= 30+(25*2)) {
            return 30+(25*2) - matchTime;
        }
        // Shift 3
        else if (matchTime <= 30+(25*3)) {
            return 30+(25*3) - matchTime;
        }
        // Shift 4
        else if (matchTime <= 30+(25*4)) {
            return 30+(25*4) - matchTime;
        }
        // Endgame
        else {
            return 30+(25*4)+30 - matchTime;
        }
    }
}
