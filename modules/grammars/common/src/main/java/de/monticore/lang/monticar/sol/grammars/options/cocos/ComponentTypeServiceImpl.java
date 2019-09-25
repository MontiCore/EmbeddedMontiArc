/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import com.google.inject.Singleton;
import org.apache.commons.io.IOUtils;
import org.json.JSONObject;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;

@Singleton
public class ComponentTypeServiceImpl implements ComponentTypeService {
    protected final JSONObject types;

    protected ComponentTypeServiceImpl() throws IOException {
        InputStream input = this.getClass().getResourceAsStream("/ComponentTypes.json");
        String content = IOUtils.toString(input, StandardCharsets.UTF_8);

        this.types = new JSONObject(content);
    }

    @Override
    public boolean isTypeRegistered(String componentType) {
        return this.types.has(componentType);
    }

    @Override
    public boolean supportsOptions(String componentType) {
        return this.getComponentType(componentType).getBoolean("options");
    }

    @Override
    public boolean supportsProp(String componentType, String prop) {
        return this.getComponentProps(componentType).keySet().contains(prop);
    }

    @Override
    public boolean propSupportsType(String componentType, String prop, String type) {
        return this.getProp(componentType, prop).getString("type").equals(type);
    }

    @Override
    public Set<String> getRequiredProps(String componentType) {
        JSONObject props = this.getComponentProps(componentType);
        Predicate<String> predicate = prop -> props.getJSONObject(prop).getBoolean("required");

        return props.keySet().stream().filter(predicate).collect(Collectors.toSet());
    }

    protected JSONObject getComponentType(String componentType) {
        return this.types.getJSONObject(componentType);
    }

    protected JSONObject getComponentProps(String componentType) {
        return this.getComponentType(componentType).getJSONObject("props");
    }

    protected JSONObject getProp(String componentType, String prop) {
        return this.getComponentProps(componentType).getJSONObject(prop);
    }
}
