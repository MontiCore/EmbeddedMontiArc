/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.symboltable.CommonSymbol;

/**
 * TODO Do we really need this?
 *
 */
public abstract class EMAAComponentImplementationSymbol extends CommonSymbol {

  public static final EMAAComponentImplementationKind KIND = EMAAComponentImplementationKind.INSTANCE;

  /**
   * Constructor for de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EMAAComponentImplementationSymbol
   *
   * @param name
   */
  public EMAAComponentImplementationSymbol(String name) {
    super(name, KIND);
  }

}
