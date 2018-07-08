package de.monticore.lang.monticar.generator.testing;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.lang.monticar.ts.references.CommonMCTypeReference;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;

import java.util.Collection;

/**
 * @author Sascha Schneiders
 */
public class StreamTestGenerator {
    protected StreamTest currentGeneratedStreamTest;

    public void createStreamTest(ExpandedComponentInstanceSymbol expandedComponentInstanceSymbol) {
        //TODO write stream test creation based on input ports with random values in input range
        currentGeneratedStreamTest = new StreamTest();
        Collection<PortSymbol> inPorts = expandedComponentInstanceSymbol.getIncomingPorts();

        for (PortSymbol p : inPorts) {
            System.out.println(p.getTypeReference().getClass().getName());
            if (p.getTypeReference().getName().equals("CommonMatrixType")) {
                //TODO handle commonMatrixType
                MCASTTypeSymbolReference typeReference = (MCASTTypeSymbolReference) p.getTypeReference();
                System.out.println(typeReference.getActualTypeArguments().size());
                if (typeReference.getAstNode().isPresent())
                    System.out.println(p.getTypeReference().getAstNode().get().getClass().getName());
            } else if (p.getTypeReference().getName().equals("Q")) {
                CommonMCTypeReference typeReference = (CommonMCTypeReference) p.getTypeReference();
                System.out.println(typeReference.getReferencedSymbol().getName());
                MontiCarTypeSymbol typeSymbol = (MontiCarTypeSymbol) typeReference.getReferencedSymbol();
                System.out.println(typeSymbol.getFormalTypeParameters().size());
                System.out.println(typeSymbol.getInterfaces().size());
                if (typeReference.getAstNode().isPresent())
                    System.out.println(p.getTypeReference().getAstNode().get().getClass().getName());
            } else if (p.getTypeReference().getName().equals("Z")) {
                CommonMCTypeReference typeReference = (CommonMCTypeReference) p.getTypeReference();
                if (typeReference.getAstNode().isPresent())
                    System.out.println(p.getTypeReference().getAstNode().get().getClass().getName());
            } else if (p.getTypeReference().getName().equals("N")) {
                CommonMCTypeReference typeReference = (CommonMCTypeReference) p.getTypeReference();
                if (typeReference.getAstNode().isPresent())
                    System.out.println(p.getTypeReference().getAstNode().get().getClass().getName());
            }
            System.out.println();
        }
    }

    public StreamTest getCurrentGeneratedStreamTest() {
        return currentGeneratedStreamTest;
    }

}
