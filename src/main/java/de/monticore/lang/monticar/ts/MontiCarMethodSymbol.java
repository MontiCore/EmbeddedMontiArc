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

public class MontiCarMethodSymbol extends CommonMCMethodSymbol<MontiCarTypeSymbol, MontiCarTypeSymbolReference, MontiCarFieldSymbol> {
    public static final MontiCarMethodSymbolKind KIND = new MontiCarMethodSymbolKind();

    private boolean isNative = false;

    private boolean isSynchronized;

    private boolean isStrictfp;

    private List<MontiCarTypeSymbolReference> annotations = new ArrayList<>();

    public MontiCarMethodSymbol(String name, MCMethodSymbolKind kind) {
        super(name, kind);
    }

    public boolean isNative() {
        return this.isNative;
    }

    public void setNative(boolean isNative) {
        this.isNative = isNative;
    }

    public boolean isSynchronized() {
        return this.isSynchronized;
    }

    public void setSynchronized(boolean isSynchronized) {
        this.isSynchronized = isSynchronized;
    }

    public boolean isStrictfp() {
        return this.isStrictfp;
    }

    public void setStrictfp(boolean isStrictfp) {
        this.isStrictfp = isStrictfp;
    }

    public void addAnnotation(MontiCarTypeSymbolReference annotation) {
        annotations.add(annotation);
    }

    public List<MontiCarTypeSymbolReference> getAnnotations() {
        return ImmutableList.copyOf(annotations);
    }

    public static class MontiCarMethodSymbolKind extends MCMethodSymbolKind {

        private static final String NAME = MontiCarMethodSymbolKind.class.getCanonicalName();

        protected MontiCarMethodSymbolKind() {
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
