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
