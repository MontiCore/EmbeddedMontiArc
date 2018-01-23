package de.monticar.lang.monticar.generator.python;

import com.esotericsoftware.yamlbeans.YamlException;
import com.esotericsoftware.yamlbeans.YamlReader;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

//TODO: replace with dependency: EMAM2PythonRos!
public class TagReader<T> {
    public List<T> readYAML(String path) {
        List<T> components;
        try {
            YamlReader reader = new YamlReader(new FileReader(path));
            components = (List<T>) reader.read();
        } catch (FileNotFoundException e) {
            return new ArrayList<>();
        } catch (YamlException e) {
            return new ArrayList<>();
        }
        return components;
    }
}