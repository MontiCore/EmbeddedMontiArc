/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos.schema;

import org.apache.commons.io.IOUtils;
import org.json.JSONObject;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.List;

/**
 * A Schema is the abstract representation of what kind of attributes are allowed/required in a tool model element.
 */
public class Schema {
    protected static final String ATTRIBUTES_KEY = "attributes";
    protected static final String TYPE_KEY = "type";
    protected static final String REQUIRED_KEY = "required";

    protected final JSONObject object;

    public Schema(String schemaPath) throws IOException {
        String normalizedSchemaPath = schemaPath.startsWith("/") ? schemaPath : String.format("/%s", schemaPath);
        InputStream input = this.getClass().getResourceAsStream(normalizedSchemaPath);
        String content = IOUtils.toString(input, StandardCharsets.UTF_8);

        this.object = new JSONObject(content);
    }

    /**
     * Returns the attributes that have to be in the model element.
     * @return The attributes that have to be in the model element.
     */
    public List<Object> getRequiredAttributes() {
        return this.object.getJSONArray(REQUIRED_KEY).toList();
    }

    /**
     * Checks whether a given attribute is supported.
     * @param attribute The attribute to be checked against.
     * @return True if the attribute is supported, false otherwise.
     */
    public boolean hasAttribute(String attribute) {
        return this.object.getJSONObject(ATTRIBUTES_KEY).has(attribute);
    }

    /**
     * Fetches the type of a given attribute.
     * @param attribute The attribute for which the type should be fetched.
     * @return The fetched type of a given attribute.
     */
    public String getTypeOfAttribute(String attribute) {
        return this.object.getJSONObject(ATTRIBUTES_KEY).getJSONObject(attribute).getString(TYPE_KEY);
    }
}
