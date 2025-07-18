/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.viewmodel;

import de.monticore.lang.monticar.enumlang._symboltable.EnumConstantDeclarationSymbol;
import de.monticore.lang.monticar.enumlang._symboltable.EnumDeclarationSymbol;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class EnumViewModel extends ViewModelBase {

    private String includeName = "";
    private List<String> constants = Collections.emptyList();

    public String getIncludeName() {
        return includeName;
    }

    public void setIncludeName(String includeName) {
        this.includeName = includeName;
    }

    public List<String> getConstants() {
        return constants;
    }

    public void setConstants(List<String> constants) {
        this.constants = constants;
    }

    public static EnumViewModel fromSymbol(EnumDeclarationSymbol s) {
        EnumViewModel vm = new EnumViewModel();
        vm.setIncludeName(Utils.getIncludeName(s));
        List<String> c = new ArrayList<>();
        for (EnumConstantDeclarationSymbol ec : s.getEnumConstants()) {
            c.add(ec.getName());
        }
        vm.setConstants(c);
        return vm;
    }
}
