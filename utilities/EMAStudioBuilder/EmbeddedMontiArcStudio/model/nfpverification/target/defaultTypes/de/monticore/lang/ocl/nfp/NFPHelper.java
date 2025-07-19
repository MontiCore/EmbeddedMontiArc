package de.monticore.lang.ocl.nfp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ElementInstance;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.montiarc.tagging._symboltable.TagKind;
import de.monticore.lang.montiarc.tagging._symboltable.TagSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewConnectorSymbol;
import org.jscience.physics.amount.Amount;

import javax.measure.quantity.Quantity;
import java.util.Collection;
import java.util.List;
import java.util.Set;
/**
 * method names for default types
 */
public class NFPHelper  {

    public static Amount max();
    public static Amount min();
    public static Amount sum();
    public static Set<ElementInstance> graph();
    public static List<List<ElementInstance>> paths();
    public static List<List<ElementInstance>> directSubComponentPaths();
}