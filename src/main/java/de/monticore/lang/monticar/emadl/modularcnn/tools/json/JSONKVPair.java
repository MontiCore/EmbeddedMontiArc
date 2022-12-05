/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.tools.json;

class JSONKVPair {
    public JSONKVPair(String key, String value, LevelType type) {
        this.key = key;
        this.value = value;
        this.type = type;
    }

    public String key;
    public String value;
    public LevelType type;
}