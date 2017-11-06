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
package de.monticore.lang.monticar.si._symboltable;

import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCMethodSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class SIUnitRangesSymbol extends CommonSymbol implements MCTypeSymbol {
    public static final SIUnitRangesSymbolKind KIND = SIUnitRangesSymbolKind.INSTANCE;

    protected List<ASTRange> ranges = new ArrayList<ASTRange>();

    public SIUnitRangesSymbol(String name) {
        super(name, KIND);
    }

    public SIUnitRangesSymbol(String name, List<ASTRange> ranges) {
        super(name, KIND);
        this.ranges.addAll(ranges);
    }

    public ASTRange getRange(int index) {
        if (0 > index || index >= ranges.size())
            Log.error("INDEX IS WRONG");
        return ranges.get(index);
    }

    public int howManyRanges() {
        return ranges.size();
    }

    //Methods below are required as SIUnitRangesSymbol is a MCTypeSymbol so
    //the already existing Port Type functionality is consistent
    //TODO change methods if they are actually needed when dealing with SIUnitRangesSymbol

    @Override
    public boolean isGeneric() {
        return false;
    }

    @Override
    public List<? extends MCTypeSymbol> getFormalTypeParameters() {
        return null;
    }

    @Override
    public Optional<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperClass() {
        return null;
    }

    @Override
    public List<? extends MCTypeReference<? extends MCTypeSymbol>> getInterfaces() {
        return null;
    }

    @Override
    public List<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperTypes() {
        return null;
    }

    @Override
    public List<? extends MCFieldSymbol> getFields() {
        return null;
    }

    @Override
    public Optional<? extends MCFieldSymbol> getField(String s) {
        return null;
    }

    @Override
    public List<? extends MCMethodSymbol> getMethods() {
        return null;
    }

    @Override
    public Optional<? extends MCMethodSymbol> getMethod(String s) {
        return null;
    }

    @Override
    public List<? extends MCMethodSymbol> getConstructors() {
        return null;
    }

    @Override
    public List<? extends MCTypeSymbol> getInnerTypes() {
        return null;
    }

    @Override
    public Optional<? extends MCTypeSymbol> getInnerType(String s) {
        return null;
    }

    @Override
    public boolean isAbstract() {
        return false;
    }

    @Override
    public boolean isFinal() {
        return false;
    }

    @Override
    public boolean isInterface() {
        return false;
    }

    @Override
    public boolean isEnum() {
        return false;
    }

    @Override
    public boolean isClass() {
        return true;
    }

    @Override
    public boolean isInnerType() {
        return false;
    }

    @Override
    public boolean isPrivate() {
        return false;
    }

    @Override
    public boolean isProtected() {
        return false;
    }

    @Override
    public boolean isPublic() {
        return false;
    }

    @Override
    public boolean isFormalTypeParameter() {
        return false;
    }

    @Override
    public Scope getSpannedScope() {
        return null;
    }
}
