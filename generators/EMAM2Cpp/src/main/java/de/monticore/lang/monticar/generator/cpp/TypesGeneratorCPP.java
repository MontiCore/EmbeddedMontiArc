/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.enumlang._symboltable.EnumDeclarationSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.monticore.lang.monticar.generator.cpp.viewmodel.EnumViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.StructViewModel;
import de.monticore.lang.monticar.struct._symboltable.StructFieldDefinitionSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public final class TypesGeneratorCPP {

    public static final String TYPES_DIRECTORY_NAME = "types";

    private List<FileContent> files;

    public List<FileContent> generateTypes(Collection<MCTypeSymbol> typeSymbols) {
        Set<String> processedTypes = new HashSet<>();
        files = new ArrayList<>();
        for (MCTypeSymbol s : typeSymbols) {
            if (processedTypes.add(s.getFullName())) {
                processSymbol(s);
            }
        }
        return Collections.unmodifiableList(files);
    }

    private void processSymbol(MCTypeSymbol s) {
        if (s instanceof StructSymbol) {
            processStruct((StructSymbol) s);
        } else if (s instanceof EnumDeclarationSymbol) {
            processEnum((EnumDeclarationSymbol) s);
        } else {
            //Log.warn("unknown type symbol: " + s.getFullName());
        }
    }

    private void processStruct(StructSymbol s) {
        for (StructFieldDefinitionSymbol sfd : s.getStructFieldDefinitions()) {
            processSymbol(sfd.getType().getReferencedSymbol());
        }
        StructViewModel vm = StructViewModel.fromSymbol(s);
        files.add(new FileContent(AllTemplates.generateStruct(vm), TYPES_DIRECTORY_NAME + "/" + vm.getIncludeName() + ".h"));
    }

    private void processEnum(EnumDeclarationSymbol s) {
        EnumViewModel vm = EnumViewModel.fromSymbol(s);
        files.add(new FileContent(AllTemplates.generateEnum(vm), TYPES_DIRECTORY_NAME + "/" + vm.getIncludeName() + ".h"));
    }
}
