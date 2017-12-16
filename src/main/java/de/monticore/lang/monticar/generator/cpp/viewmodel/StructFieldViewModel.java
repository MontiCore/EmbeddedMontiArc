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
        if ("Q".equals(fullName)) {
            vm.setType("double");
            vm.setInitializer("0.0");
        } else if ("B".equals(fullName)) {
            vm.setType("bool");
            vm.setInitializer("false");
        } else if ("Z".equals(fullName)) {
            vm.setType("int");
            vm.setInitializer("0");
        } else if (ts instanceof StructSymbol) {
            vm.setType(Utils.getIncludeName(ts));
        } else if (ts instanceof EnumDeclarationSymbol) {
            vm.setType(Utils.getIncludeName(ts));
        } else {
            Log.error("unknown type: " + ts.getFullName());
        }
        return vm;
    }
}
