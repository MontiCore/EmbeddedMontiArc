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
public class MultiParamValueSymbol extends ValueSymbol {
    public static final MultiParamValueSymbolKind KIND = new MultiParamValueSymbolKind();

    private Map<String, Object> parameters;

    public MultiParamValueSymbol() {
        super("", KIND);
        this.parameters = new HashMap<>();
    }

    public Map<String, Object> getParameters() {
        return parameters;
    }

    public Object getParameter(final String parameterName) {
        return parameters.get(parameterName);
    }

    public boolean hasParameter(final String parameterName) {
        return parameters.containsKey(parameterName);
    }

    public void addParameter(final String parameterName, final Object value) {
        parameters.put(parameterName, value);
    }

    @Override
    public String toString() {
        return super.toString() + '{' + parameters + '}';
    }
}
