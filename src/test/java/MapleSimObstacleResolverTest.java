import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.MapleSimObstacleResolver;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.Test;

public class MapleSimObstacleResolverTest {
    @Test
    public void testAutoStartUnclipping() {
        double bumperLengthX = Units.Inches.of(34.5).in(Units.Meters);
        double bumperWidthY = Units.Inches.of(34.5).in(Units.Meters);
        Convex bumper = Geometry.createRectangle(bumperLengthX, bumperWidthY);

        Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
        MapleSimObstacleResolver.installNonStickyContactListener(arena);

        // C-Shoot / C-Depot start waypoint is at (3.6, 4.0), facing 180 degrees
        // The Hub front face is at X = 4.0005 m.
        // With half-width 0.438 m, robot bumper extends to X = 4.038 m (penetrating 3.8 cm into the Hub).
        Pose2d requestedPose = new Pose2d(3.6, 4.0, Rotation2d.fromDegrees(180));

        Pose2d resolvedPose = MapleSimObstacleResolver.resolvePoseClipping(requestedPose, bumper, arena);
        System.out.println("Requested Pose: " + requestedPose);
        System.out.println("Resolved Pose:  " + resolvedPose);

        // The resolved pose must shift in -X direction to clear the Hub
        assertTrue(resolvedPose.getX() < requestedPose.getX(), "Robot must shift away from the Hub");

        // Running resolvePoseClipping again must produce no further shift (already clean)
        Pose2d secondPass = MapleSimObstacleResolver.resolvePoseClipping(resolvedPose, bumper, arena);
        assertEquals(resolvedPose.getX(), secondPass.getX(), 1e-4, "Second pass must produce no X shift");
        assertEquals(resolvedPose.getY(), secondPass.getY(), 1e-4, "Second pass must produce no Y shift");
    }
}
