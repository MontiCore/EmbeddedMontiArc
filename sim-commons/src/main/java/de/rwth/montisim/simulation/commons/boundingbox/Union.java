package de.rwth.montisim.simulation.commons.boundingbox;

import java.util.ArrayList;
import java.util.List;

public class Union implements BoundingBox {
    public List<BoundingBox> boundingBoxes = new ArrayList<>();

    @Override
    public boolean collidesWith(BoundingBox other) {
        // When type is not specified -> use visitor pattern using this class
        return other.collidesWith(this);
    }

    @Override
    public boolean collidesWith(AABB other) {
        for (BoundingBox bb : boundingBoxes) {
            if (bb.collidesWith(other)) return true;
        }
        return false;
    }

    @Override
    public boolean collidesWith(OBB other) {
        for (BoundingBox bb : boundingBoxes) {
            if (bb.collidesWith(other)) return true;
        }
        return false;
    }

    @Override
    public boolean collidesWith(ConvexHull other) {
        for (BoundingBox bb : boundingBoxes) {
            if (bb.collidesWith(other)) return true;
        }
        return false;
    }

    @Override
    public boolean collidesWith(Union other) {
        for (BoundingBox bb : boundingBoxes) {
            if (bb.collidesWith(other)) return true;
        }
        return false;
    }
}
