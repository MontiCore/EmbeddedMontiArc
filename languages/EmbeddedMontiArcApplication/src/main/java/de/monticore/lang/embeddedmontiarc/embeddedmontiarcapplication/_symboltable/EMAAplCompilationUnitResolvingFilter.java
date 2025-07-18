/* (c) https://github.com/MontiCore/monticore */



package de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class EMAAplCompilationUnitResolvingFilter extends CommonResolvingFilter<EMAAplCompilationUnitSymbol> {

    public EMAAplCompilationUnitResolvingFilter() {
        super(EMAAplCompilationUnitSymbol.KIND);
    }
}
