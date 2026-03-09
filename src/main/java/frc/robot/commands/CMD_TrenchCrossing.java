package frc.robot.commands;

import java.util.Set;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.CommandSwerveDrivetrain;

public class CMD_TrenchCrossing {
    private static final String[] TRENCH_PATH_NAMES = {
        "Left Trough Center - Neutral Zone Left",
        "Neutral Zone Left - Left Trough Center",
        "Right Trough Center - Neutral Zone Right",
        "Neutral Zone Right - Right Trough Center "
    };

    public static Command create(CommandSwerveDrivetrain drivetrain) {
        return new DeferredCommand(() -> {
            try {
                Pose2d currentPose = drivetrain.getPose();
                PathPlannerPath nearestPath = null;
                double minDistance = Double.MAX_VALUE;

                for (String pathName : TRENCH_PATH_NAMES) {
                    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                    var startPoseOptional = path.getStartingHolonomicPose();

                    if (startPoseOptional.isEmpty()) {
                        continue;
                    }

                    Pose2d startPose = startPoseOptional.get();
                    double distance = currentPose.getTranslation().getDistance(startPose.getTranslation());
                    if (distance < minDistance) {
                        minDistance = distance;
                        nearestPath = path;
                    }
                }

                if (nearestPath == null) return Commands.none();

                // Pathfind to the start of the nearest trench path, then follow it
                return AutoBuilder.pathfindThenFollowPath(
                    nearestPath, 
                    new PathConstraints(4.0, 3.0, 360, 540)
                );
            } catch (Exception e) {
                DriverStation.reportError("Failed to load trench paths: " + e.getMessage(), e.getStackTrace());
                return Commands.none();
            }
        }, Set.of(drivetrain));
    }
}
