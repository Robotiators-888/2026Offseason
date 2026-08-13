package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.CMD_AimBot;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.Notification.NotificationLevel;

public class Hub {
    private static Boolean lastActiveAlliance = true;
    private static String gameData = "";

    /** AprilTag on the face of each alliance's hub. */
    private static final int kRedHubTagId = 10;
    private static final int kBlueHubTagId = 26;

    /**
     * Distance from the hub's face tag to the hub's center, along +X. Signed so that it always
     * points from the tag into the middle of the field.
     */
    private static final double kTagToHubCenterMeters = Units.inchesToMeters(23.5);

    /** @return The AprilTag ID on our alliance's hub. Defaults to blue when the alliance is unknown. */
    public static int getGoalTagId () {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? kRedHubTagId
                : kBlueHubTagId;
    }

    /**
     * Field-relative translation of the center of our alliance's hub, derived from the AprilTag
     * layout rather than hardcoded coordinates.
     *
     * @return The hub center, or empty if the tag is missing from the loaded layout.
     */
    public static Optional<Translation2d> getGoalTranslation () {
        final boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        final double offsetX = isRed ? -kTagToHubCenterMeters : kTagToHubCenterMeters;

        // Read the layout directly rather than through SUB_PhotonVision.getInstance(), which would
        // construct camera objects as a side effect of asking a geometry question.
        return Constants.Field.kTagLayout.getTagPose(getGoalTagId())
                .map(Pose3d::toPose2d)
                .map(pose -> pose.getTranslation().plus(new Translation2d(offsetX, 0)));
    }

    /**
     * Distance from a robot pose to the center of our alliance's hub.
     *
     * <p>Returns empty rather than a silent zero when the hub tag is unavailable, so callers must
     * decide what to do instead of aiming at themselves.
     *
     * @param robotPose Current robot pose on the field
     * @return Distance in meters, or empty if the hub tag is missing from the layout
     */
    public static Optional<Double> getDistanceToGoal (final Pose2d robotPose) {
        return getGoalTranslation().map(goal -> robotPose.getTranslation().getDistance(goal));
    }

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