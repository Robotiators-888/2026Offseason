package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.dyn4j.collision.narrowphase.Gjk;
import org.dyn4j.collision.narrowphase.Penetration;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.contact.Contact;
import org.dyn4j.dynamics.contact.ContactConstraint;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.ContactCollisionData;
import org.dyn4j.world.listener.ContactListenerAdapter;
import org.ironmaple.utils.mathutils.GeometryConvertor;

/**
 * Utility for resolving simulation clipping and preventing sticky-obstacle traps in MapleSim.
 */
public final class MapleSimObstacleResolver {
    private static final Gjk GJK_DETECTOR = new Gjk();
    private static final double SAFETY_MARGIN_METERS = 0.003; // 3mm clearance to prevent contact re-trigger
    private static final int MAX_UNCLIP_ITERATIONS = 8;
    private static boolean listenerInstalled = false;

    private MapleSimObstacleResolver() {}

    /**
     * Installs a contact listener on the simulation world that zeroes obstacle friction.
     * Prevents the robot from getting glued / pinned to walls or obstacles.
     *
     * @param arena The active SimulatedArena instance.
     */
    public static synchronized void installNonStickyContactListener(SimulatedArena arena) {
        if (arena == null || listenerInstalled) {
            return;
        }

        arena.physicsWorld.addContactListener(new ContactListenerAdapter<Body>() {
            @Override
            public void preSolve(ContactCollisionData<Body> collision, Contact contact) {
                ContactConstraint<Body> constraint = collision.getContactConstraint();
                if (constraint != null) {
                    Body b1 = constraint.getBody1();
                    Body b2 = constraint.getBody2();
                    if ((b1 != null && b1.getMass().isInfinite()) || (b2 != null && b2.getMass().isInfinite())) {
                        constraint.setFriction(0.0);
                    }
                }
            }
        });

        listenerInstalled = true;
    }

    /**
     * Checks if the requested robot pose causes its bumper to penetrate any static field obstacle,
     * and automatically shifts the pose outward along the penetration normal until it is clear.
     *
     * @param requestedPose The desired robot pose (e.g. from AutoBuilder starting waypoint).
     * @param bumperShape The convex collision shape of the robot bumper.
     * @param arena The active SimulatedArena instance.
     * @return A depenetrated Pose2d cleanly resting outside all obstacles.
     */
    public static Pose2d resolvePoseClipping(Pose2d requestedPose, Convex bumperShape, SimulatedArena arena) {
        if (requestedPose == null || bumperShape == null || arena == null) {
            return requestedPose;
        }

        Pose2d currentPose = requestedPose;

        for (int iter = 0; iter < MAX_UNCLIP_ITERATIONS; iter++) {
            Transform robotTransform = GeometryConvertor.toDyn4jTransform(currentPose);
            double maxPenetrationDepth = 0.0;
            Vector2 bestSeparationVector = null;

            int bodyCount = arena.physicsWorld.getBodyCount();
            for (int i = 0; i < bodyCount; i++) {
                Body body = arena.physicsWorld.getBody(i);
                if (body != null && body.getMass().isInfinite()) {
                    int fixtureCount = body.getFixtureCount();
                    for (int j = 0; j < fixtureCount; j++) {
                        var fixture = body.getFixture(j);
                        Penetration p = new Penetration();
                        if (GJK_DETECTOR.detect(bumperShape, robotTransform, fixture.getShape(), body.getTransform(), p)) {
                            if (p.getDepth() > maxPenetrationDepth) {
                                maxPenetrationDepth = p.getDepth();
                                bestSeparationVector = p.getNormal().product(-(p.getDepth() + SAFETY_MARGIN_METERS));
                            }
                        }
                    }
                }
            }

            if (bestSeparationVector == null || maxPenetrationDepth < 1e-4) {
                break;
            }

            currentPose = new Pose2d(
                currentPose.getX() + bestSeparationVector.x,
                currentPose.getY() + bestSeparationVector.y,
                currentPose.getRotation()
            );
        }

        return currentPose;
    }
}
