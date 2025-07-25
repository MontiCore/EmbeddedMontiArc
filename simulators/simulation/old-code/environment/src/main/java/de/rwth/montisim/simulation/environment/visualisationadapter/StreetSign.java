/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.visualisationadapter;

import de.rwth.montisim.commons.utils.Vec3;

public class StreetSign {

    public static final long ID = -1;

    private SignTypeAndState type;

    private SignTypeAndState state;

    private Vec3 p1;
    private Vec3 p2;

    private boolean isOne;
    private boolean isTwo;


    public StreetSign(SignTypeAndState type) {
        this.type = type;
        this.isOne = this.isTwo = false;
        if (type != SignTypeAndState.TRAFFIC_LIGHT) {
            state = type;
        }
    }

    public SignTypeAndState getSignState() {
        return this.state;
    }

    public SignTypeAndState getType() {
        return this.type;
    }

    public long getId() {
        return StreetSign.ID;
    }

    public boolean isOne() {
        return this.isOne;
    }

    public boolean isTwo() {
        return this.isTwo;
    }

    public void setOne(Vec3 p1) {
        this.p1 = p1;
        this.isOne = (p1 != null);
    }

    public void setTwo(Vec3 p2) {
        this.p2 = p2;
        this.isTwo = (p2 != null);
    }
}
