/**
 *
 *  ******************************************************************************
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

package Bus;


import commons.controller.commons.BusEntry;
import simulation.simulator.*;

import java.awt.*;
import java.util.*;
import java.lang.Math;
import java.util.List;
import java.util.ArrayList;
import java.util.Optional;
import java.lang.IllegalArgumentException;


public class InstantBus implements Bus {
    private CanBus bus;
    public InstantBus() {
        bus = new CanBus(1);
    }

    @overried
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime){
        bus.didExecuteLoop(simulationObjects, totalTime, deltaTime);
    }

}