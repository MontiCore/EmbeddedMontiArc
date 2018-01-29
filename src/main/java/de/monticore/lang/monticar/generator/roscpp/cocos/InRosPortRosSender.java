package de.monticore.lang.monticar.generator.roscpp.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.monticar.generator.roscpp.tagging.RosConnectionSymbol;
import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class InRosPortRosSender implements EmbeddedMontiArcASTComponentCoCo {
    @Override
    public void check(ASTComponent node) {
        InRosPortRosSenderChecker check = new InRosPortRosSenderChecker();
//        check.check(node);
        node.get_Children().stream()
                .flatMap(n -> n.get_Children().stream())
                .flatMap(n -> n.get_Children().stream())
                .flatMap(n -> n.get_Children().stream())
                .flatMap(n -> n.get_Children().stream())
                .filter(n -> n instanceof ASTTaggingUnit)
                .map(n -> (ASTTaggingUnit) n)
                .forEach(astTaggingUnit -> {
                    System.out.println(astTaggingUnit);
                });


    }


    class InRosPortRosSenderChecker {
        private TaggingResolver taggingResolver;

        void check(ASTComponent node) {
            //TODO: add check: connectors with ros ports as target only have ros ports as source(and topic matches)
            node.getPorts().forEach(astPort -> {
                astPort.get_Children().stream()
                        .filter(n -> n.getSymbol().isPresent())
                        .map(n -> n.getSymbol().get())
                        .filter(s -> s.isKindOf(RosConnectionSymbol.KIND))
                        .map(s -> (RosConnectionSymbol) s)
                        .forEach(rosConnectionSymbol -> {
                            System.out.println(rosConnectionSymbol);
                        });
            });

        }

    }
}
