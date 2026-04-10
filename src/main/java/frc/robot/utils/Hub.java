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
    // Used for publishing and rumble logic
    private static Boolean lastActiveAlliance = true;
    // Holds cached gamedata from the fms that tells us who won auto
    private static String gameData = "";

    // Tells us if our hub is currently active
    // Its a convinient wrapper around getActiveAlliance
    // Returns empty if autonomous or gamedata is empty after fetching it
    public static Optional<Boolean> isAllianceHubActive () {
        if (getActiveAlliance().isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(getActiveAlliance().get() == DriverStation.getAlliance().orElse(Alliance.Red));
    }

    // Gets the active alliance
    // Returns empty if autonomous
    public static Optional<Alliance> getActiveAlliance () {
        // Return empty if autonomous
        if (DriverStation.isAutonomous()) {
            return Optional.empty();
        }
        
        // Try to fetch match data if not cached, return empty if this fails
        double matchTime = DriverStation.getMatchTime();
        if (gameData.length() == 0) {
            gameData = DriverStation.getGameSpecificMessage(); 
            if (gameData.length() == 0) {
                return Optional.empty();
            }
        }
        
        // Determine the alliance that wins autonmous
        Alliance initialAlliance = gameData.charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;

        // Return the correct info based on match time and initialAlliance
        // If in alliance shift or endgame, return our alliance
        if (matchTime >= 130 || matchTime < 30) {
            return Optional.of(DriverStation.getAlliance().orElse(Alliance.Red));
        }
        // In the correct shifts return the opposite of initial alliance
        else if (matchTime >= 105 || (matchTime < 80 && matchTime >= 55)) {
            return initialAlliance == Alliance.Red ? Optional.of(Alliance.Blue) : Optional.of(Alliance.Red);
        }
        // In other shifts return the initial alliance
        else {
            return Optional.of(initialAlliance);
        }
    }

    // Gets the time until next alliance change which is helpful for rumble and elastic publishing
    public static double getTimeUntilNextChange () {
        double matchTime = DriverStation.getMatchTime();
        // Auto and transition shift
        if (matchTime >= 30+(25*4)) {
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

    // Not needed but precaches the gamedata, it should be called on teleop init
    public static void fetchMatchData () {
        gameData = DriverStation.getGameSpecificMessage();
    }

    // Used to rumble the controllers
    public static void start(CommandXboxController Driver1, CommandXboxController Driver2, SUB_Shooter shooter) {
        final Optional<Boolean> activeAlliance = Hub.isAllianceHubActive();
        SmartDashboard.putBoolean("Hub/Last Active Alliance", lastActiveAlliance);
        if (activeAlliance.isPresent() && lastActiveAlliance != activeAlliance.get()) {
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Active hub change",
                                "The active hub has changed!"));
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