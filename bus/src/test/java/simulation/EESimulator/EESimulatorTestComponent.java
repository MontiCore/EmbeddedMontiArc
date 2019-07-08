package simulation.EESimulator;
/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */


import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import commons.controller.commons.BusEntry;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEDiscreteEvent;
import simulation.EESimulator.EESimulator;
import simulation.bus.BusMessage;

public class EESimulatorTestComponent extends EEComponent {

    private UUID ID;

    public EESimulatorTestComponent(EESimulator simulator, ArrayList<BusEntry> listenTo) {
        super(simulator);
        this.ID = UUID.randomUUID();
        for (BusEntry entry : listenTo ) {
            this.listenTo.add(entry);
        }
    }

    @Override
    public void processEvent(EEDiscreteEvent event) {
        ((BusMessage) event).setMessage(this.ID.toString() + " processed");

    }

    @Override
    public UUID getID() {
        return ID;
    }

}
