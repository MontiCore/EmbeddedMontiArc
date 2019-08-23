/* (c) https://github.com/MontiCore/monticore */
package de.rwth_aachen.se.montisim.simulators.basic_simulator.controller;

import commons.controller.interfaces.Bus;

import java.util.HashMap;
import java.util.Map;

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
