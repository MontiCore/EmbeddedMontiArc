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

import java.util.List;
import java.util.Map;
import java.util.Optional;

public interface Bus {
    
    public void registerComponent(Object component);

    public void registerData(String key, BusMessage msg);

    public Optional<BusMessage> getData(String key);

    public Map<String, BusMessage> getAllData();

    public String[] getImportNames();
    
    /**
     * @param startTime start of the simulation in microseconds
     * @param duration duration of the simulation in microseconds
     * @return
     */
    public List<BusMessage> simulateFor(int startTime, int duration);
}
