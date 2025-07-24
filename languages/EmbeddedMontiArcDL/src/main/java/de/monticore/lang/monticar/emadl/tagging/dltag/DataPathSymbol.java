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

public class DataPathSymbol extends TagSymbol {
    public static final DataPathKind KIND  = DataPathKind.INSTANCE;

    public DataPathSymbol() {
        super(KIND, ".");
    }

    public DataPathSymbol(String path, String type) {
        this(KIND, path, type);
    }

    public DataPathSymbol(DataPathKind kind, String path, String type) {
        super(kind, path, type);
    }

    public String getPath() {
      return getValue(0);
    }

    public String getType() {
      return getValue(1);
    }

    @Override
    public String toString() {
        return super.toString();
    }

    public static class DataPathKind extends TagKind {
        public static final DataPathKind INSTANCE = new DataPathKind();

        protected DataPathKind() {
        }
    }
}
