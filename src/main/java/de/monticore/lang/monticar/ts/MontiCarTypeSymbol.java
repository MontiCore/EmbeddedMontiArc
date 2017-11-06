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
package de.monticore.lang.monticar.ts;

import com.google.common.collect.ImmutableList;
import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;
import de.monticore.symboltable.SymbolKind;

import java.util.ArrayList;
import java.util.List;

public class MontiCarTypeSymbol extends
        CommonMCTypeSymbol<MontiCarTypeSymbol, MontiCarFieldSymbol, MontiCarMethodSymbol, MontiCarTypeSymbolReference> {
    public static final MontiCarTypeSymbolKind KIND = new MontiCarTypeSymbolKind();

    private boolean isStatic;

    private boolean isStrictfp;

    private boolean isAnnotation;

    private boolean isTypeVariable;

    private List<MontiCarTypeSymbolReference> annotations = new ArrayList<>();

    protected MontiCarTypeSymbol(
            String name,
            MCTypeSymbolKind typeKind,
            MCAttributeSymbolKind attributeKind,
            MCMethodSymbolKind methodKind) {
        super(name, typeKind, attributeKind, methodKind);
    }

    public MontiCarTypeSymbol(String name) {
        this(name, MontiCarTypeSymbol.KIND, MontiCarFieldSymbol.KIND, MontiCarMethodSymbol.KIND);
    }

    public boolean isStatic() {
        return this.isStatic;
    }

    public void setStatic(boolean isStatic) {
        this.isStatic = isStatic;
    }

    public boolean isStrictfp() {
        return this.isStrictfp;
    }

    public void setStrictfp(boolean isStrictfp) {
        this.isStrictfp = isStrictfp;
    }

    public boolean isAnnotation() {
        return this.isAnnotation;
    }

    public void setAnnotation(boolean isAnnotation) {
        this.isAnnotation = isAnnotation;
    }

    public boolean isTypeVariable() {
        return this.isTypeVariable;
    }

    public void setTypeVariable(boolean isTypeVariable) {
        this.isTypeVariable = isTypeVariable;
    }

    public void addAnnotation(MontiCarTypeSymbolReference annotation) {
        annotations.add(annotation);
    }

    public List<MontiCarTypeSymbolReference> getAnnotations() {
        return ImmutableList.copyOf(annotations);
    }

    public static class MontiCarTypeSymbolKind extends MCTypeSymbolKind {

        private static final String NAME = MontiCarTypeSymbolKind.class.getCanonicalName();

        protected MontiCarTypeSymbolKind() {
        }

        @Override
        public String getName() {
            return NAME;
        }

        @Override
        public boolean isKindOf(SymbolKind kind) {
            return NAME.equals(kind.getName()) || super.isKindOf(kind);
        }
    }
}
