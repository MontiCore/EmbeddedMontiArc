/*
 * ******************************************************************************
 * MontiCore Language Workbench, www.monticore.de
 * Copyright (c) 2017, MontiCore, All rights reserved.
 *
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * ******************************************************************************
 */

package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import de.se_rwth.commons.logging.Log;
import org.eclipse.core.internal.jobs.ObjectMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Contains all information that is relevant for the target language to generate the source code.
 *
 * @author Sascha Schneiders
 */
public abstract class LanguageUnit {

    protected List<BluePrint> bluePrints = new ArrayList<>();
    protected List<Symbol> symbolsToConvert = new ArrayList<>();

    public List<BluePrint> getBluePrints() {
        return bluePrints;
    }

    public void addSymbolToConvert(Symbol symbol) {
        symbolsToConvert.add(symbol);
    }

    public Optional<BluePrint> getBluePrint(String fullName) {
        for (BluePrint bluePrint : bluePrints) {
            if (bluePrint.getName().equals(fullName.replaceAll("\\.", "_"))) {
                return Optional.of(bluePrint);
            }
        }
        return Optional.empty();
    }

    public abstract void generateBluePrints();

}
