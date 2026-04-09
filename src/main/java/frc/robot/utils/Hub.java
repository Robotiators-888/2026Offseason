package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CMD_AimBot;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.Notification.NotificationLevel;

public class Hub {
    private static Boolean lastActiveAlliance = true;
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
        // Auto and transition shift
        if (matchTime >= 30+(25*4)) {
            // Idk if this is right
            return 30 - (30+(25*4)+30 - matchTime);
        }
        // Shift 1
        else if (matchTime >= 30+(25*3)) {
            return 25 - (30+(25*4) - matchTime);
        }
        // Shift 2
        else if (matchTime >= 30+(25*2)) {
            return 25 - (30+(25*3) - matchTime);
        }
        // Shift 3
        else if (matchTime >= 30+(25*1)) {
            return 25 - (30+(25*2) - matchTime);
        }
        // Shift 4
        else if (matchTime >= 30) {
            return 25 - (30+(25*1) - matchTime);
        }
        // Endgame
        else {
            return 30 - (30 - matchTime);
        }
    }

    public static void fetchMatchData () {
        gameData = DriverStation.getGameSpecificMessage();
    }

    public static void start(CommandXboxController Driver1, CommandXboxController Driver2, SUB_Shooter shooter) {
        final Optional<Boolean> activeAlliance = Hub.isAllianceHubActive();
        SmartDashboard.putBoolean("Hub/Last Active Alliance", lastActiveAlliance);
        if (activeAlliance.isPresent() && lastActiveAlliance != activeAlliance.get()) {
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Active hub change",
                                "The active hub has changed!"));
                // Maybe do a rumble
                lastActiveAlliance = activeAlliance.get();
        }
        SmartDashboard.putNumber("Hub/Time until next alliance change", Hub.getTimeUntilNextChange());
        if (Hub.isAllianceHubActive().isPresent()) {
                SmartDashboard.putBoolean("Hub/Is our Alliance Active", Hub.isAllianceHubActive().get());
        }
        if ((Hub.getTimeUntilNextChange() <= 3.25 && Hub.getTimeUntilNextChange() >= 2.75)
                        || (Hub.getTimeUntilNextChange() <= 2.25 && Hub.getTimeUntilNextChange() >= 1.75)
                        || (Hub.getTimeUntilNextChange() <= 1.25 && Hub.getTimeUntilNextChange() >= 0.75)) {
                Driver1.getHID().setRumble(RumbleType.kLeftRumble, 1);
                Driver2.getHID().setRumble(RumbleType.kLeftRumble, 1);
        } else {
                Driver1.getHID().setRumble(RumbleType.kLeftRumble, 0);
                Driver2.getHID().setRumble(RumbleType.kLeftRumble, 0);
        }
        if (shooter.atDesiredRPM() && CMD_AimBot.isThetaErrorCorrect && CMD_AimBot.isRunning()) {
                Driver1.getHID().setRumble(RumbleType.kRightRumble, 1);
                Driver2.getHID().setRumble(RumbleType.kRightRumble, 1);
        } else {
                Driver1.getHID().setRumble(RumbleType.kRightRumble, 0);
                Driver2.getHID().setRumble(RumbleType.kRightRumble, 0);
        }
    }
}