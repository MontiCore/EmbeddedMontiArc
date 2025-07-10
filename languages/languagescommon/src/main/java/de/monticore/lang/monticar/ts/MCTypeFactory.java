/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

public interface MCTypeFactory<U extends CommonMCTypeSymbol<?, ?>> {
    U createTypeVariable(String var1);
}
