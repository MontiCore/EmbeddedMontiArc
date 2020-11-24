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

public class LayerArtifactParameterSymbol extends TagSymbol {

    public static final LayerArtifactParameterKind KIND  = LayerArtifactParameterKind.INSTANCE;

    public LayerArtifactParameterSymbol() {
        super(KIND, ".");
    }

    public LayerArtifactParameterSymbol(String artifact, String name) {
        this(KIND, artifact, name);
    }

    public LayerArtifactParameterSymbol(LayerArtifactParameterKind kind, String artifact, String name) {
        super(kind, artifact, name);
    }

    public String getArtifact() {
      return getValue(0);
    }

    public String getId() {
        return getValue(1);
    }

    @Override
    public String toString() {
        return super.toString();
    }

    public static class LayerArtifactParameterKind extends TagKind {
        public static final LayerArtifactParameterKind INSTANCE = new LayerArtifactParameterKind();

        protected LayerArtifactParameterKind() {
        }
    }
}
