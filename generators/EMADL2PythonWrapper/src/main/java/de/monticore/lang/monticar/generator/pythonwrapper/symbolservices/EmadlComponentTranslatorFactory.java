/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices;

/**
 *
 */
public class EmadlComponentTranslatorFactory {
    public EmadlComponentTranslator create() {
        EmadlInstanceSymbolUtil symbolUtil = new EmadlInstanceSymbolUtil();
        EmadlInstanceSymbol2PortVariableMapper instance2PortVariableMapper
                = new EmadlInstanceSymbol2PortVariableMapper(symbolUtil);
        return new EmadlComponentTranslator(instance2PortVariableMapper);
    }
}
