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

package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import java.util.Optional;

import de.monticore.symboltable.types.JTypeSymbol;
import de.monticore.symboltable.types.references.JTypeReference;
import de.se_rwth.commons.logging.Log;


public class EMAPortBuilder {
    protected Optional<Boolean> incoming = Optional.empty();
    protected Optional<String> name = Optional.empty();
    protected Optional<JTypeReference<? extends JTypeSymbol>> typeReference = Optional.empty();

    public static PortSymbol clone(PortSymbol port) {
        return new EMAPortBuilder().setName(port.getName()).setDirection(port.isIncoming()).setTypeReference(port.getTypeReference()).build();
    }

    public EMAPortBuilder setDirection(boolean incoming) {
        this.incoming = Optional.of(Boolean.valueOf(incoming));
        return this;
    }

    public EMAPortBuilder setName(String name) {
        this.name = Optional.of(name);
        return this;
    }

    public EMAPortBuilder setTypeReference(Optional<JTypeReference<? extends JTypeSymbol>> typeReference) {
        this.typeReference = typeReference;
        return this;
    }

    public PortSymbol build() {
        if (name.isPresent() && incoming.isPresent()) {
            PortSymbol p = new PortSymbol(this.name.get());
            p.setDirection(this.incoming.get());
            p.setTypeReference(this.typeReference);
            return p;
        }
        Log.error("not all parameters have been set before to build the port symbol");
        throw new Error("not all parameters have been set before to build the port symbol");
    }

//    public ConstantPortSymbol buildConstantPort(JTypeReference typeReference) {
//        if (typeReference == null) {
//            Log.error("not all parameters have been set before to build the port symbol");
//            throw new Error("not all parameters have been set before to build the port symbol");
//        }
//        ConstantPortSymbol p = new ConstantPortSymbol();
//        p.setDirection(false);
//        p.setTypeReference(typeReference);
//        return p;
//    }
}
