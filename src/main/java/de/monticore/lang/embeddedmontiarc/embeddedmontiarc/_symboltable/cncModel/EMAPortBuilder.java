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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantValue;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;


public class EMAPortBuilder {
    protected Optional<Boolean> incoming = Optional.empty();
    protected Optional<String> name = Optional.empty();
    protected Optional<MCTypeReference> typeReference = Optional.empty();
    protected Optional<EMAConstantValue> constantValue = Optional.empty();
    protected Optional<ASTNode> astNode = Optional.empty();
    protected Optional<Boolean> config = Optional.empty();
    protected Optional<MiddlewareSymbol> middlewareSymbol = Optional.empty();

    public static EMAPortSymbol clone(EMAPortSymbol port) {
        if (port.isConstant())
            return new EMAPortBuilder().setName(port.getName()).setDirection(port.isIncoming()).
                    setTypeReference(port.getTypeReference()).setConstantValue(((EMAConstantPortSymbol) port).getConstantValue()).setASTNode(port.getAstNode())
                    .buildConstantPort();
        else {
            return new EMAPortBuilder().setName(port.getName()).setDirection(port.isIncoming())
                    .setTypeReference(port.getTypeReference()).setASTNode(port.getAstNode()).setConfig(port.isConfig()).setMiddlewareSymbol(port.getMiddlewareSymbol()).build();
        }
    }

    public EMAPortBuilder setDirection(boolean incoming) {
        this.incoming = Optional.of(Boolean.valueOf(incoming));
        return this;
    }

    public EMAPortBuilder setConfig(boolean config) {
        this.config = Optional.of(config);
        return this;
    }

    public EMAPortBuilder setConstantValue(EMAConstantValue constantValue) {
        this.constantValue = Optional.of(constantValue);
        return this;
    }

    public EMAPortBuilder setName(String name) {
        this.name = Optional.of(name);
        return this;
    }

    public EMAPortBuilder setASTNode(Optional<ASTNode> astNode) {
        this.astNode = astNode;
        return this;
    }
    public EMAPortBuilder setMiddlewareSymbol(Optional<MiddlewareSymbol> middlewareSymbol){
        this.middlewareSymbol = middlewareSymbol;
        return this;
    }


    public EMAPortBuilder setTypeReference(MCTypeReference typeReference) {
        this.typeReference = Optional.of(typeReference);
        return this;
    }

    public EMAPortSymbol build() {
        if (name.isPresent() && incoming.isPresent() && typeReference.isPresent()) {
            EMAPortSymbol p = new EMAPortSymbol(this.name.get());
            p.setDirection(this.incoming.get());
            p.setTypeReference(this.typeReference.get());
            if (astNode.isPresent())
                p.setAstNode(astNode.get());
            if (config.isPresent())
                p.setConfig(config.get());
            if(middlewareSymbol.isPresent())
                p.setMiddlewareSymbol(middlewareSymbol.get());
            return p;
        }
        Log.error("not all parameters have been set before to build the port symbol");
        throw new Error("not all parameters have been set before to build the port symbol");
    }

    public EMAConstantPortSymbol buildConstantPort() {
        if (typeReference == null) {
            Log.error("not all parameters have been set before to build the port symbol");
            throw new Error("not all parameters have been set before to build the port symbol");
        }
        EMAConstantPortSymbol p = new EMAConstantPortSymbol(name.get());
        p.setDirection(this.incoming.get());
        p.setTypeReference(typeReference.get());
        p.setConstantValue(constantValue.get());
        if (astNode.isPresent())
            p.setAstNode(astNode.get());
        if (config.isPresent())
            p.setConfig(config.get());
        if(middlewareSymbol.isPresent())
            p.setMiddlewareSymbol(middlewareSymbol.get());
        return p;
    }


    public static EMAPortInstanceSymbol instantiate(EMAPortSymbol port) {
        EMAPortInstanceSymbol portInstance = new EMAPortInstanceSymbol(port.getName());
        portInstance.setDirection(port.isIncoming());
        portInstance.setTypeReference(port.getTypeReference());
        if (port.getAstNode().isPresent())
            portInstance.setAstNode(port.getAstNode().get());
        portInstance.setConfig(port.isConfig());
        if(port.getMiddlewareSymbol().isPresent())
            portInstance.setMiddlewareSymbol(port.getMiddlewareSymbol().get());
        return portInstance;
    }
}
