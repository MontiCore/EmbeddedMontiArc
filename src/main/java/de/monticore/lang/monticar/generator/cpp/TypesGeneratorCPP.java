/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
            Log.warn("unknown type symbol: " + s.getFullName());
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
