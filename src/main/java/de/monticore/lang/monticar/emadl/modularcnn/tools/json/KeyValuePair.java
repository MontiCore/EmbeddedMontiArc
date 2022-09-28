package de.monticore.lang.monticar.emadl.modularcnn.tools.json;

class KeyValuePair {
    public KeyValuePair(String key, String value, LevelType type) {
        this.key = key;
        this.value = value;
        this.type = type;
    }

    public String key;
    public String value;
    public LevelType type;
}