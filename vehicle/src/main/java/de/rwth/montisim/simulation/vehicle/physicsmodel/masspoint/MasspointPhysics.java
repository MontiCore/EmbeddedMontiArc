/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel.masspoint;

import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;

public class MasspointPhysics implements PhysicsModel {
    public static final String TYPE = "masspoint";
    DynamicObject physObj;

    public MasspointPhysics(PowerTrain power_train, EESystem eesimulator) {

    }

    @Override
    public DynamicObject getPhysicalObject() {
        return physObj;
    }

    @Override
    public void update(TimeUpdate timeUpdate) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setGroundPosition(Vec3 pos, Vec2 front) {
        // TODO Auto-generated method stub

    }

    @Override
    public PhysicsProperties getProperties() {
        // TODO Auto-generated method stub
        return null;
    }

}