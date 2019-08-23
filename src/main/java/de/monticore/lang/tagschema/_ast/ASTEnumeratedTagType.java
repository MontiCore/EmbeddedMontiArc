/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.tagschema._ast;

import java.util.LinkedHashSet;
import java.util.Optional;
import java.util.Set;

/**
 * Created by MichaelvonWenckstern on 15.06.2016.
 */
public class ASTEnumeratedTagType extends ASTEnumeratedTagTypeTOP {
    protected ASTEnumeratedTagType() {
        super();
    }

    protected Set<String> enumValues = new LinkedHashSet<>();

    protected ASTEnumeratedTagType(String name, String enumText, Optional<ASTScope> scope) {
        super(name, enumText, scope);
        this.enumValues = enumValues;
    }

    public void setEnumText(String enumText) {
        if (enumText != null) {
            if (enumText.startsWith("[")) {
                enumText = enumText.substring(1);
            }
            if (enumText.endsWith("]")) {
                enumText = enumText.substring(0, enumText.length() - 1);
            }
            enumText = enumText.trim();
            // TODO fix: now it also splits BC and D in A | "BC|D" | X
            String vs[] = enumText.split("\\|");
            for (String v : vs) {
                enumValues.add(v.trim());
            }
        }
        super.setEnumText(enumText);
    }

    public Set<String> getEnumValues() {
        return enumValues;
    }
}
