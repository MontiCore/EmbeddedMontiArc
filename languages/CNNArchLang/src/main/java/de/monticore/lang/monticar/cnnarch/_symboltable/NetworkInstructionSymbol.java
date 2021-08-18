/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.SymbolKind;

import java.util.*;

public abstract class NetworkInstructionSymbol extends ResolvableSymbol {

    private SerialCompositeElementSymbol body;

    protected NetworkInstructionSymbol(String name, SymbolKind kind) {
        super(name, kind);
    }
    
    public SerialCompositeElementSymbol getBody() {
        return body;
    }

    protected void setBody(SerialCompositeElementSymbol body) {
        this.body = body;
    }

    public boolean containsAdaNet(){
        return this.body.containsAdaNet();
    }
    public boolean isStream() {
        return false;
    }

    public boolean isUnroll() {
        return false;
    }

    public StreamInstructionSymbol toStreamInstruction() {
        return (StreamInstructionSymbol) this;
    }

    public UnrollInstructionSymbol toUnrollInstruction() {
        return (UnrollInstructionSymbol) this;
    }

}
