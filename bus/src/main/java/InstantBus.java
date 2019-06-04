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
///**
// *
// * ******************************************************************************
// *  MontiCAR Modeling Family, www.se-rwth.de
// *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
// *  All rights reserved.
// *
// *  This project is free software; you can redistribute it and/or
// *  modify it under the terms of the GNU Lesser General Public
// *  License as published by the Free Software Foundation; either
// *  version 3.0 of the License, or (at your option) any later version.
// *  This library is distributed in the hope that it will be useful,
// *  but WITHOUT ANY WARRANTY; without even the implied warranty of
// *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// *  Lesser General Public License for more details.
// *
// *  You should have received a copy of the GNU Lesser General Public
// *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
// * *******************************************************************************
// */
//package Bus;
//
//
//import commons.controller.commons.BusEntry;
//import simulation.simulator.*;
//
//import java.awt.*;
//import java.util.*;
//import java.lang.Math;
//
//
//public class InstantBus implements Bus {
//    private CanBus bus;
//    public InstantBus() {
//        bus = new CanBus(1, Optional.empty);
//    }
//
//    @overried
//    public void registerData(String key, BusMessage msg) {
//        msg.setFinishTime(msg.getRequestTime);
//        bus.registerData(key, msg);
//    }
//
//    @override
//    public BusMessage getData(String key) {
//        return bus.getData(key);
//    }
//
//    @override
//    public Map<String, BusMessage> getAllData() {
//        return bus.getAllData();
//    }
//
//    @override
//    public String[] getImportNames() {
//        bus.getImportNames();
//    }
//
//    @override
//    public void registerComponent(Object component) {
//        bus.registerComponent(component);
//    }
//
//    /**
//     * simulate the transmission of data for a given time
//     *
//     * @param startTime time when simulation starts
//     * @param duration  time of the simulation
//     * @return list of transmitted messages
//     */
//    @override
//    public List<BusMessage> simulateFor(int startTime, int duration) {
//
//        return bus.simulateFor(startTime, duration);
//    }
//
//}