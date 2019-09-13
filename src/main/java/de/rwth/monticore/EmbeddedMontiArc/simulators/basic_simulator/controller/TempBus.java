/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller;

import java.util.HashMap;
import java.util.Map;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces.Bus;

public class TempBus implements Bus {
    private Map<String, Object> busMap= new HashMap<String, Object>();;
    @Override
    public void setData(String key, Object object) {
        busMap.put(key, object);
    }

    @Override
    public void setAllData(Map<String, Object> map) {
        busMap.putAll(map);
    }

    @Override
    public Object getData(String key) {
        return busMap.get(key);
    }

    @Override
    public Map<String, Object> getAllData() {
        return busMap;
    }

    @Override
    public String[] getImportNames() {
        return new String[0];
    }
}
