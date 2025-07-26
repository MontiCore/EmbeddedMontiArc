/* (c) https://github.com/MontiCore/monticore */
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
            return new EMAPortBuilder()
                    .setName(port.getName())
                    .setDirection(port.isIncoming())
                    .setTypeReference(port.getTypeReference())
                    .setConstantValue(port.getConstantValue().orElse(null))
                    .setMiddlewareSymbol(port.getMiddlewareSymbol().orElse(null))
                    .setASTNode(port.getAstNode())
                    .setConfig(port.isConfig())
                    .build();
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
        this.constantValue = Optional.ofNullable(constantValue);
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
    public EMAPortBuilder setMiddlewareSymbol(MiddlewareSymbol middlewareSymbol){
        this.middlewareSymbol = Optional.ofNullable(middlewareSymbol);
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
            if(constantValue.isPresent())
                p.setConstantValue(constantValue.get());
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

    public static EMAPortInstanceSymbol instantiate(EMAPortSymbol port, String packageName, String name) {
        EMAPortInstanceSymbol portInstance = new EMAPortInstanceSymbol(name);
        portInstance.setDirection(port.isIncoming());
        portInstance.setTypeReference(port.getTypeReference());
        if (port.getAstNode().isPresent())
            portInstance.setAstNode(port.getAstNode().get());
        portInstance.setConfig(port.isConfig());
        if(port.getMiddlewareSymbol().isPresent())
            portInstance.setMiddlewareSymbol(port.getMiddlewareSymbol().get());
        if(port.getConstantValue().isPresent())
            portInstance.setConstantValue(port.getConstantValue().get());
        portInstance.setPackageName(packageName);
        portInstance.setFullName(packageName + "." + port.getName());
        return portInstance;
    }
}
