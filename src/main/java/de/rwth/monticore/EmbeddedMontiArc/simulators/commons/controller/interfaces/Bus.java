/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces;

import java.util.Map;

/**
 * This is an interface for the DataBus.
 *
 * Created by Christoph Grüne on 30.12.2016.
 */
public interface Bus {

    /**
     * puts a specific value to a key
     *
     * @param key is the key
     * @param object is the object
     */
    public abstract void setData(String key, Object object);

    /**
     * integrates a existing map into this one
     *
     * @param map is the map
     */
    public void setAllData(Map<String, Object> map);

    /**
     * returns the object of a certain key
     *
     * @param key is the key of the object
     * @return the object with key key
     */
    public Object getData(String key);

    /**
     * returns the connectionMap map
     *
     * @return busMap the Map that contains all information
     */
    public Map<String, Object> getAllData();

    /**
     * returns all import names for maps
     *
     * @return all import names
     */
    public abstract String[] getImportNames();
}
