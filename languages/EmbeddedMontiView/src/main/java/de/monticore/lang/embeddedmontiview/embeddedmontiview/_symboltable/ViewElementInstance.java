/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;

import java.util.Collection;

/**
 * Created by kt on 27.02.2017.
 */
public interface ViewElementInstance {

    public Collection<CommonSymbol> getTags();
    public <T extends CommonSymbol> Collection<T> getTags(final CommonSymbol tagKind);
    public String getName();
    public Scope getEnclosingScope() ;
}
