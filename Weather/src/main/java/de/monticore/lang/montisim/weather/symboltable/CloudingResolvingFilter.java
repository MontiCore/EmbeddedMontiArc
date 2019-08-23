/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class CloudingResolvingFilter extends CommonResolvingFilter<CloudingSymbol> {
  public CloudingResolvingFilter() {
    super(CloudingSymbol.KIND);
  }
}
