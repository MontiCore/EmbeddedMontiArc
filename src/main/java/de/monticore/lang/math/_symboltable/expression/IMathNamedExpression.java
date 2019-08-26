/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

/**
 * @author Sascha Schneiders
 */
public interface IMathNamedExpression {
    String getNameToAccess();
    void setNameToAccess(String nameToAccess);
}
