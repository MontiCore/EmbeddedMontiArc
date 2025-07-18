/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.tagging.dltag;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

public class LayerPathParameterSymbol extends TagSymbol {
    public static final LayerPathParameterKind KIND  = LayerPathParameterKind.INSTANCE;
    public LayerPathParameterSymbol() {
        super(KIND, ".");
    }

    public LayerPathParameterSymbol(String path, String name) {
        this(KIND, path, name);
    }

    public LayerPathParameterSymbol(LayerPathParameterKind kind, String path, String name) {
        super(kind, path, name);
    }

    public String getPath() {
      return getValue(0);
    }

    public String getId() {
        return getValue(1);
    }

    @Override
    public String toString() {
        return super.toString();
    }

    public static class LayerPathParameterKind extends TagKind {
        public static final LayerPathParameterKind INSTANCE = new LayerPathParameterKind();

        protected LayerPathParameterKind() {
        }
    }
}
