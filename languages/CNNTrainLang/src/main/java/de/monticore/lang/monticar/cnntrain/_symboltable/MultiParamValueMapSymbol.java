/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import java.util.HashMap;
import java.util.Map;

/**
 *
 */
public class MultiParamValueMapSymbol extends ValueSymbol {
    public static final MultiParamValueMapSymbolKind KIND = new MultiParamValueMapSymbolKind();

    private Map<String, Map<String,Object>> parameters;
    private Map<String, String> multiParamValueNames;

    public MultiParamValueMapSymbol() {
        super("", KIND);
        this.parameters = new HashMap<>();
        this.multiParamValueNames = new HashMap<>();
    }

    public Map<String, Map<String,Object>> getParameters() {
        return parameters;
    }

    public Map<String, String> getMultiParamValueNames() { return multiParamValueNames; }

    public Object getParameter(final String parameterName) {
        return parameters.get(parameterName);
    }

    public boolean hasParameter(final String parameterName) {
        return parameters.containsKey(parameterName);
    }

    public void addParameter(final String parameterName, final Map<String, Object> value) {
        parameters.put(parameterName, value);
    }

    public void addMultiParamValueName(final String parameterName, final String name) {
        multiParamValueNames.put(parameterName, name);
    }

    @Override
    public String toString() {
        return super.toString() + '{' + parameters + '}';
    }
}
