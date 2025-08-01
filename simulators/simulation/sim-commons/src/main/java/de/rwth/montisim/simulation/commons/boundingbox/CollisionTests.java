package de.rwth.montisim.simulation.commons.boundingbox;

import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec3;

public class CollisionTests {

    public static boolean collides(OBB obb, ConvexHull hull) {
        // Test OBB against half-spaces of the Hull
        for (int i = 0; i < hull.halfSpaceNormals.length; ++i) {
            Vec3 normal = hull.halfSpaceNormals[i];
            if (IPM.dot(obb.pos, normal) -
                    (Math.abs(IPM.dot(obb.world_space_half_axes.col1, normal)) +
                            Math.abs(IPM.dot(obb.world_space_half_axes.col2, normal)) +
                            Math.abs(IPM.dot(obb.world_space_half_axes.col3, normal)))
                    >= hull.halfSpacePos[i]) return false;
        }

        // Test hull points against half-axes of the vehicle
        if (isSeparated(obb.pos, obb.world_space_half_axes.col1, hull)) return false;
        if (isSeparated(obb.pos, obb.world_space_half_axes.col2, hull)) return false;
        if (isSeparated(obb.pos, obb.world_space_half_axes.col3, hull)) return false;

        // TODO every halfAxis cross halfSpaceNormals combinations should also be checked

        return true;
    }

    static private boolean isSeparated(Vec3 pos, Vec3 axis, ConvexHull hull) {
        double min = IPM.dot(axis, axis);
        double ref = IPM.dot(axis, pos);
        boolean hasNeg = false;
        boolean hasPos = false;
        for (Vec3 corner : hull.corners) {
            double res = ref - IPM.dot(axis, corner);
            // Check that all vertices were on the "same side"
            if (res < 0.0) {
                hasNeg = true;
                if (hasPos) return false;
            } else {
                hasPos = true;
                if (hasNeg) return false;
            }
            if (Math.abs(res) < min) return false;
        }
        return true;
    }

    // From https://stackoverflow.com/questions/47866571/simple-oriented-bounding-box-obb-collision-detection-explaining
    // test for separating planes in all 15 axes
    public static boolean collides(OBB a, OBB b, Vec3 tempVec1, Vec3 tempVec2) {
        Vec3 RPos = tempVec1;
        Vec3 crossRes = tempVec2;

        IPM.subtractTo(RPos, a.pos, b.pos);

        if (getSeparatingPlane(a.world_space_half_axes.col1, a, b, RPos)) return false;
        if (getSeparatingPlane(a.world_space_half_axes.col2, a, b, RPos)) return false;
        if (getSeparatingPlane(a.world_space_half_axes.col3, a, b, RPos)) return false;
        if (getSeparatingPlane(b.world_space_half_axes.col1, a, b, RPos)) return false;
        if (getSeparatingPlane(b.world_space_half_axes.col2, a, b, RPos)) return false;
        if (getSeparatingPlane(b.world_space_half_axes.col3, a, b, RPos)) return false;

        IPM.crossTo(crossRes, a.world_space_half_axes.col1, b.world_space_half_axes.col1);
        if (getSeparatingPlane(crossRes, a, b, RPos)) return false;
        IPM.crossTo(crossRes, a.world_space_half_axes.col1, b.world_space_half_axes.col2);
        if (getSeparatingPlane(crossRes, a, b, RPos)) return false;
        IPM.crossTo(crossRes, a.world_space_half_axes.col1, b.world_space_half_axes.col3);
        if (getSeparatingPlane(crossRes, a, b, RPos)) return false;
        IPM.crossTo(crossRes, a.world_space_half_axes.col2, b.world_space_half_axes.col1);
        if (getSeparatingPlane(crossRes, a, b, RPos)) return false;
        IPM.crossTo(crossRes, a.world_space_half_axes.col2, b.world_space_half_axes.col2);
        if (getSeparatingPlane(crossRes, a, b, RPos)) return false;
        IPM.crossTo(crossRes, a.world_space_half_axes.col2, b.world_space_half_axes.col3);
        if (getSeparatingPlane(crossRes, a, b, RPos)) return false;
        IPM.crossTo(crossRes, a.world_space_half_axes.col3, b.world_space_half_axes.col1);
        if (getSeparatingPlane(crossRes, a, b, RPos)) return false;
        IPM.crossTo(crossRes, a.world_space_half_axes.col3, b.world_space_half_axes.col2);
        if (getSeparatingPlane(crossRes, a, b, RPos)) return false;
        IPM.crossTo(crossRes, a.world_space_half_axes.col3, b.world_space_half_axes.col3);
        if (getSeparatingPlane(crossRes, a, b, RPos)) return false;

        return true;
    }

    // check if there's a separating plane in between the selected axes
    private static boolean getSeparatingPlane(Vec3 Plane, OBB a, OBB b, Vec3 RPos) {
        return Math.abs(IPM.dot(RPos, Plane)) >
                (Math.abs(IPM.dot(a.world_space_half_axes.col1, Plane)) +
                        Math.abs(IPM.dot(a.world_space_half_axes.col2, Plane)) +
                        Math.abs(IPM.dot(a.world_space_half_axes.col3, Plane)) +
                        Math.abs(IPM.dot(b.world_space_half_axes.col1, Plane)) +
                        Math.abs(IPM.dot(b.world_space_half_axes.col2, Plane)) +
                        Math.abs(IPM.dot(b.world_space_half_axes.col3, Plane)));
    }
}
