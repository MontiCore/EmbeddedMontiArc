/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicPortArraySymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.kind.EMADynamicPortInstanceKIND;
import de.monticore.symboltable.SymbolKind;

import java.util.Optional;

public class EMADynamicPortInstanceSymbol extends EMAPortInstanceSymbol {

    public static final EMADynamicPortInstanceKIND KIND = EMADynamicPortInstanceKIND.INSTANCE;

    protected boolean dynamic;

    protected boolean dimensionInfinite;

    protected Optional<String> nameSizeDependsOn = Optional.empty();

    public EMADynamicPortInstanceSymbol(String name) {
        super(name, KIND);
        dynamic = false;
        dimensionInfinite = false;
    }

    protected EMADynamicPortInstanceSymbol(String name, SymbolKind kind) {
        super(name, kind);
        dynamic = false;
        dimensionInfinite = false;
    }

    public boolean isDynamic() {
        return dynamic;
    }
    public void setDynamic(boolean dynamic) {
        this.dynamic = dynamic;
    }

    public void instantiate(EMAPortSymbol port, String packageName){
        //this should be in a parent class
        this.setDirection(port.isIncoming());
        this.setTypeReference(port.getTypeReference());
        if (port.getAstNode().isPresent())
            this.setAstNode(port.getAstNode().get());
        this.setConfig(port.isConfig());
        if(port.getMiddlewareSymbol().isPresent())
            this.setMiddlewareSymbol(port.getMiddlewareSymbol().get());
        if(port.getConstantValue().isPresent())
            this.setConstantValue(port.getConstantValue().get());
        this.setPackageName(packageName);
        this.setFullName(packageName + "." + port.getName());

        if(port instanceof EMADynamicPortArraySymbol){
            this.setDynamic(((EMADynamicPortArraySymbol)port).isDynamic());
        }

//        this.nameDependsOn = port.getNameDependsOn();
    }

    public static EMADynamicPortInstanceSymbol newAndInstantiate(EMAPortSymbol port, String packageName){
        EMADynamicPortInstanceSymbol p = new EMADynamicPortInstanceSymbol(port.getName());
        p.instantiate(port, packageName);
        return p;
    }

    public static EMADynamicPortInstanceSymbol newAndInstantiate(EMADynamicPortArraySymbol port, String packageName){
        EMADynamicPortInstanceSymbol p = new EMADynamicPortInstanceSymbol(port.getName());
        p.instantiate(port, packageName);
        return p;
    }

    public boolean isDimensionInfinite() {
        return dimensionInfinite;
    }

    public void setDimensionInfinite(boolean dimensionInfinite) {
        this.dimensionInfinite = dimensionInfinite;
    }


    public Optional<String> getNameSizeDependsOn() {
        return nameSizeDependsOn;
    }

    public void setNameSizeDependsOn(Optional<String> nameSizeDependsOn) {
        this.nameSizeDependsOn = nameSizeDependsOn;
    }

    @Override
    public String getFullName() {
        return determineFullName();
    }
}
