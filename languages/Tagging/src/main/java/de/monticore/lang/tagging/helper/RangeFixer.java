/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.tagging.helper;

import com.google.common.collect.Lists;
import de.monticore.lang.tagging._ast.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class RangeFixer {
    private RangeFixer() {

    }

    public static void fixTaggingUnit(ASTTaggingUnit astTaggingUnit) {
        //Replace all range syntax scopes with expanded versions
        astTaggingUnit.getTagBody().getTagList()
                .forEach(tag -> {
                    List<ASTScope> fixedScopes = tag.getScopeList().stream()
                            .map(RangeFixer::expandScopeWithRange)
                            .flatMap(Collection::stream)
                            .collect(Collectors.toList());

                    tag.setScopeList(fixedScopes);
                });
    }

    public static List<ASTScope> expandScopeWithRange(ASTScope scope) {
        if (scope.getScopeKind().equals("NameScope")) {
            ASTNameScope nameScope = (ASTNameScope) scope;
            List<ASTQualifiedNameWithArray> qualifiedNames = new ArrayList<>();
            //add empty start name
            qualifiedNames.add(TaggingMill.qualifiedNameWithArrayBuilder().build());

            for (ASTNameWithArray part : nameScope.getQualifiedName().getPartsList()) {
                List<ASTNameWithArray> expandedParts = expandRangeInPart(part);
                List<ASTQualifiedNameWithArray> expandedNames = new ArrayList<>();
                for (ASTNameWithArray ep : expandedParts) {
                    for (ASTQualifiedNameWithArray name : qualifiedNames) {
                        ASTQualifiedNameWithArray tmpName = name.deepClone();
                        tmpName.getPartsList().add(ep);
                        expandedNames.add(tmpName);
                    }
                }
                qualifiedNames = expandedNames;
            }

            return qualifiedNames.stream().map(name -> TaggingMill.nameScopeBuilder().setQualifiedName(name).build()).collect(Collectors.toList());

        } else {
            return Lists.newArrayList(scope);
        }
    }

    public static List<ASTNameWithArray> expandRangeInPart(ASTNameWithArray part) {
        //Not a range
        if (!part.getStartOpt().isPresent() || !part.getEndOpt().isPresent()) {
            return Lists.newArrayList(part);
        }

        List<ASTNameWithArray> result = new ArrayList<>();

        int step = part.getStepOpt().isPresent() ? part.getStepOpt().get().getValue() : 1;
        int start = part.getStartOpt().get().getValue();
        int end = part.getEndOpt().get().getValue();

        for (int i = start; i <= end; i += step) {
            ASTNameWithArray tmpNameWithArray = TaggingMill.nameWithArrayBuilder()
                    .setName(part.getName())
                    .setStart(TaggingMill.intLiteralBuilder().setSource("" + i).build())
                    .build();
            result.add(tmpNameWithArray);
        }

        return result;
    }
}
