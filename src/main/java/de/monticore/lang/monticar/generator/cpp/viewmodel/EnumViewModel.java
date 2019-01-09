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
