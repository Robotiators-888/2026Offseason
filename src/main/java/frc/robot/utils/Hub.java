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
    // Primitive, not Boolean: the boxed version was compared with != against another boxed
    // Boolean, i.e. by reference — it only worked because Boolean.valueOf caches TRUE/FALSE.
    private static boolean lastActiveAlliance = true;
    private static String gameData = "";

    /**
     * Teleop-countdown seconds at which the active hub changes, descending: the transition shift
     * ends at 130, the four 25-second alliance shifts follow, and endgame (both hubs active)
     * starts at 30.
     */
    private static final double[] kShiftBoundaries = {130, 105, 80, 55, 30};

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
        return getActiveAlliance()
                .map(active -> active == DriverStation.getAlliance().orElse(Alliance.Red));
    }

    public static Optional<Alliance> getActiveAlliance () {
        if (DriverStation.isAutonomous()) {
            return Optional.empty();
        }
        if (gameData.length() == 0) {
            // The FMS only publishes game data ~3 s after auto ends, so keep re-fetching until
            // it appears rather than trusting the one fetch in teleopInit().
            gameData = DriverStation.getGameSpecificMessage();
        }
        return activeAllianceAt(DriverStation.getMatchTime(), gameData,
                DriverStation.getAlliance().orElse(Alliance.Red));
    }

    /**
     * The alliance whose hub is active, as a pure function so it is unit-testable.
     *
     * <p>Per the 2026 game data spec, the game-specific message is a single character naming the
     * alliance that out-scored auto, whose hub goes <b>inactive</b> first: that alliance's hub is
     * active in shifts 2 and 4, the other alliance's in shifts 1 and 3. During the transition
     * shift and endgame both hubs are active, reported here as {@code ourAlliance} so
     * "is our hub active" reads true.
     *
     * @param matchTime teleop countdown in seconds
     * @param gameData FMS game-specific message ("R"/"B"), or empty if not yet published
     * @param ourAlliance our alliance, used for the both-active phases
     */
    public static Optional<Alliance> activeAllianceAt (final double matchTime,
            final String gameData, final Alliance ourAlliance) {
        if (gameData == null || gameData.isEmpty()) {
            return Optional.empty();
        }
        if (matchTime >= kShiftBoundaries[0]
                || matchTime < kShiftBoundaries[kShiftBoundaries.length - 1]) {
            return Optional.of(ourAlliance);
        }
        final Alliance inactiveFirst = gameData.charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;
        final Alliance activeFirst = inactiveFirst == Alliance.Red ? Alliance.Blue : Alliance.Red;

        int shift = 0;
        for (final double boundary : kShiftBoundaries) {
            if (matchTime < boundary) {
                shift++;
            }
        }
        // shift is 1..4 here; odd shifts belong to the alliance whose hub is active first.
        return Optional.of(shift % 2 == 1 ? activeFirst : inactiveFirst);
    }

    public static double getTimeUntilNextChange () {
        return timeUntilNextChangeAt(DriverStation.getMatchTime());
    }

    /**
     * Seconds until the next hub activity change, as a pure function of the teleop countdown.
     * Counts down to each boundary in {@link #kShiftBoundaries} and to zero in endgame.
     */
    public static double timeUntilNextChangeAt (final double matchTime) {
        for (final double boundary : kShiftBoundaries) {
            if (matchTime >= boundary) {
                return matchTime - boundary;
            }
        }
        return matchTime;
    }

    public static void fetchMatchData () {
        gameData = DriverStation.getGameSpecificMessage();
    }

    public static void start(CommandXboxController Driver1, CommandXboxController Driver2, SUB_Shooter shooter) {
        // Computed once per call — this used to re-derive the shift state up to seven times per
        // loop, each re-reading the DriverStation, so values could disagree within one loop.
        final Optional<Boolean> activeAlliance = Hub.isAllianceHubActive();
        final double timeUntilChange = Hub.getTimeUntilNextChange();

        SmartDashboard.putBoolean("Hub/Last Active Alliance", lastActiveAlliance);
        if (activeAlliance.isPresent() && lastActiveAlliance != activeAlliance.get()) {
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Active hub change",
                                "The active hub has changed!"));
                // Maybe do a rumble
                lastActiveAlliance = activeAlliance.get();
        }
        SmartDashboard.putNumber("Hub/Time until next alliance change", timeUntilChange);
        if (activeAlliance.isPresent()) {
                SmartDashboard.putBoolean("Hub/Is our Alliance Active", activeAlliance.get());
        }
        if ((timeUntilChange <= 3.25 && timeUntilChange >= 2.75)
                        || (timeUntilChange <= 2.25 && timeUntilChange >= 1.75)
                        || (timeUntilChange <= 1.25 && timeUntilChange >= 0.75)) {
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