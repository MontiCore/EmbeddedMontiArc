/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.pedestrians;

import de.rwth.montisim.commons.utils.Vec3;

/**
 * Created by lukas on 07.02.17.
 *
 * A container for the parameters needed to compute the movement of a pedestrian
 */
public class PedestrianStreetParameters {
    private boolean isCrossing;
    private boolean direction;
    private boolean leftPavement;
    private Vec3 position;

    public PedestrianStreetParameters(boolean isCrossing, Vec3 position, boolean direction, boolean leftPavement) {
        this.isCrossing = isCrossing;
        this.position = position;
        this.direction = direction;
        this.leftPavement = leftPavement;
    }

    /**
     * @return the position of the pedestrian
     */
    public Vec3 getPosition() {
        return position;
    }

    /**
     * @return true iff the pedestrian crosses the street
     */
    public boolean isCrossing() {
        return isCrossing;
    }

    /**
     * @return true if the pedestrian walks from p3D1 to p3D2. false if the pedestrian walks from p3D2 to p3D1. might have undefined behaviour
     * while the pedestrian crosses the street
     */
    public boolean isDirection() {
        return direction;
    }

    /**
     * @return true iff the pedestrian walks on the left pavement
     */
    public boolean isLeftPavement() {
        return this.leftPavement;
    }
}
