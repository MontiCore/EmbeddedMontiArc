/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.viewmodel;

import de.monticore.lang.monticar.enumlang._symboltable.EnumDeclarationSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructFieldDefinitionSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.se_rwth.commons.logging.Log;

public class StructFieldViewModel extends ViewModelBase {

    private String name = "";
    private String type = "";
    private String initializer = "";

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    public String getInitializer() {
        return initializer;
    }

    public void setInitializer(String initializer) {
        this.initializer = initializer;
    }

    public static StructFieldViewModel fromSymbol(StructFieldDefinitionSymbol s) {
        StructFieldViewModel vm = new StructFieldViewModel();
        vm.setName(s.getName());
        MCTypeSymbol ts = s.getType().getReferencedSymbol();
        String fullName = ts.getFullName();
        handleCases(vm, fullName, ts);
        return vm;
    }

    public static void handleCases(StructFieldViewModel viewModel, String fullName, MCTypeSymbol typeSymbol) {
        if (handleDefaultTypeCases(viewModel, fullName)) {
            //handled if true
        } else if (handleSpecialTypeCases(viewModel, typeSymbol)) {
            //handled if true
        } else {
            Log.error("unknown type: " + typeSymbol.getFullName());
        }
    }

    public static boolean handleDefaultTypeCases(StructFieldViewModel viewModel, String fullName) {
        boolean handled = false;
        if ("Q".equals(fullName)) {
            viewModel.setType("double");
            viewModel.setInitializer("0.0");
            handled = true;
        } else if ("B".equals(fullName)) {
            viewModel.setType("bool");
            viewModel.setInitializer("false");
            handled = true;
        } else if ("Z".equals(fullName)) {
            viewModel.setType("int");
            viewModel.setInitializer("0");
            handled = true;
        }
        return handled;
    }

    public static boolean handleSpecialTypeCases(StructFieldViewModel viewModel, MCTypeSymbol typeSymbol) {
        boolean handled = false;
        if (typeSymbol instanceof StructSymbol) {
            viewModel.setType(Utils.getIncludeName(typeSymbol));
            handled = true;
        } else if (typeSymbol instanceof EnumDeclarationSymbol) {
            viewModel.setType(Utils.getIncludeName(typeSymbol));
            handled = true;
        }
        return handled;
    }
}
