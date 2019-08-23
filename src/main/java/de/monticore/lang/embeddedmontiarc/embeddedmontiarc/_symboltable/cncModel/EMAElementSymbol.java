/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;

import de.monticore.symboltable.Scope;

public interface EMAElementSymbol {
    public String getName();
    public Scope getEnclosingScope() ;
}
