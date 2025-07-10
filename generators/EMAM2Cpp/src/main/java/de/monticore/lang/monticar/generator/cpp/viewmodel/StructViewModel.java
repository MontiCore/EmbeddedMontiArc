/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.viewmodel;

import de.monticore.lang.monticar.enumlang._symboltable.EnumDeclarationSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructFieldDefinitionSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class StructViewModel extends ViewModelBase {

    private String includeName = "";
    private Set<String> additionalIncludes = Collections.emptySet();
    private List<StructFieldViewModel> fields = Collections.emptyList();

    public String getIncludeName() {
        return includeName;
    }

    public void setIncludeName(String includeName) {
        this.includeName = includeName;
    }

    public Set<String> getAdditionalIncludes() {
        return additionalIncludes;
    }

    public void setAdditionalIncludes(Set<String> additionalIncludes) {
        this.additionalIncludes = additionalIncludes;
    }

    public List<StructFieldViewModel> getFields() {
        return fields;
    }

    public void setFields(List<StructFieldViewModel> fields) {
        this.fields = fields;
    }

    public static StructViewModel fromSymbol(StructSymbol s) {
        StructViewModel vm = new StructViewModel();
        vm.setIncludeName(Utils.getIncludeName(s));
        Set<String> additionalIncludes = new HashSet<>();
        List<StructFieldViewModel> fields = new ArrayList<>();
        for (StructFieldDefinitionSymbol sfd : s.getStructFieldDefinitions()) {
            MCTypeSymbol fieldTypeSym = sfd.getType().getReferencedSymbol();
            if (fieldTypeSym instanceof StructSymbol || fieldTypeSym instanceof EnumDeclarationSymbol) {
                additionalIncludes.add(Utils.getIncludeName(fieldTypeSym));
            }
            fields.add(StructFieldViewModel.fromSymbol(sfd));
        }
        vm.setAdditionalIncludes(additionalIncludes);
        vm.setFields(fields);
        return vm;
    }
}
