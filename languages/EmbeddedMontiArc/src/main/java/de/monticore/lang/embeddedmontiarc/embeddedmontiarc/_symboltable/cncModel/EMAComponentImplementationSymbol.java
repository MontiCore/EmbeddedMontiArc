/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;

import de.monticore.symboltable.CommonSymbol;

/**
 * TODO Do we really need this?
 *
 */
public abstract class EMAComponentImplementationSymbol
        extends CommonSymbol {

  public static final EMAComponentImplementationKind KIND = EMAComponentImplementationKind.INSTANCE;

  /**
   * Constructor for de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentImplementationSymbol
   *
   * @param name
   */
  public EMAComponentImplementationSymbol(String name) {
    super(name, KIND);
  }

}
