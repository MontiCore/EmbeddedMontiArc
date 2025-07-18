package de.monticore.lang.montiarc.montiarc._symboltable;

import de.monticore.lang.montiarc.tagging._symboltable.TagKind;
import de.monticore.lang.montiarc.tagging._symboltable.TagSymbol;
import de.monticore.symboltable.Scope;

import java.util.Collection;

/**
 * method names for default types
 */
public interface ElementInstance {
    public String getName();
    public Scope getEnclosingScope();
    public Collection<TagSymbol> getTags();
    public <T extends TagSymbol> Collection<T> getTags(final TagKind tagKind);
}
