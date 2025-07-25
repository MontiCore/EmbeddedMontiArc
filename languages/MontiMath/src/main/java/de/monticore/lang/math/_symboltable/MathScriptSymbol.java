/* (c) https://github.com/MontiCore/monticore */
// created by Michael von Wenckstern



package de.monticore.lang.math._symboltable;

public class MathScriptSymbol extends de.monticore.symboltable.CommonScopeSpanningSymbol {

    public static final MathScriptKind KIND = new MathScriptKind();


    public MathScriptSymbol(String name) {
        super(name, KIND);
    }

}
