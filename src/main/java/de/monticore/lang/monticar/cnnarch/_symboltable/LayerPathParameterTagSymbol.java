/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.SymbolKind;
import de.monticore.symboltable.CommonSymbol;

import java.util.*;

public abstract class LayerPathParameterTagSymbol extends CommonSymbol {

    public static final LayerPathParameterTagKind KIND = new LayerPathParameterTagKind();
    private String path;
    private String id;

    protected LayerPathParameterTagSymbol(String name) {
        super(name, KIND);
    }

    public String getPath() {
        return path;
    }

    public void setPath(String path) {
        this.path = path;
    }

    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }
}
