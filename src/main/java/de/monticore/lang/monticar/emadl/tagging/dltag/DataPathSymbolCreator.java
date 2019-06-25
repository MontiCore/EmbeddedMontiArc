/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.emadl.tagging.dltag;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.tagging._ast.ASTNameScope;
import de.monticore.lang.tagging._ast.ASTTag;
import de.monticore.lang.tagging._ast.ASTTagBody;
import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.types.types._ast.ASTQualifiedName;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;

import java.util.Objects;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class DataPathSymbolCreator implements TagSymbolCreator {

    private final String regexExpression =  "\\s*\\{\\s*path\\s*=\\s*(.*)\\s*,\\s*type\\s*=\\s*(.*)\\s*\\}\\s*";
    private final Pattern pattern = Pattern.compile(regexExpression, Pattern.MULTILINE);

    @Override
    public void create(ASTTaggingUnit unit, TaggingResolver tagging) {
        boolean hasDataPathTag =
            unit.getQualifiedNameList().stream()
                .map(ASTQualifiedName::toString)
                .anyMatch(n -> n.endsWith("DataPathTagSchema"));

        if (!hasDataPathTag) {
            return;
        }

        final String packageName = Joiners.DOT.join(unit.getPackageList());
        final ASTTagBody tagBody = unit.getTagBody();
        final String root =
            (tagBody.getTargetModelOpt().isPresent()) ?
                Joiners.DOT.join(packageName, tagBody.getTargetModelOpt().get()
                    .getQualifiedNameString()) :
                packageName;


        for (ASTTag tag : unit.getTagBody().getTagList()) {
            tag.getTagElementList().stream()
                .filter(tagElement -> tagElement.getName().equals("DataPath"))
                .map(tagElement -> matchRegexPattern(tagElement.getTagValue()))
                .filter(Objects::nonNull)
                .forEachOrdered(matcher -> tag.getScopeList().stream()
                    .filter(scope -> scope.getScopeKind().equals("NameScope"))
                    .map(scope -> (ASTNameScope) scope)
                    .map(scope ->
                            tagging.resolve(dotJoin(root, scope.getQualifiedNameString()), EMAComponentSymbol.KIND)
                    )
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .forEachOrdered(scope ->
                            tagging.addTag(scope, new DataPathSymbol(matcher.group(1), matcher.group(2)))
                    ));
        }
    }

    private Matcher matchRegexPattern(String regex) {
        Matcher matcher = pattern.matcher(regex);
        if (matcher.matches()) {
            return matcher;
        }
        else {
            Log.error(
                String.format(
                        "'%s' does not match the specified regex pattern '%s'",
                        regex, "{path = {dataPath}, type = {dataType}}"
                )
            );
            return null;
        }

    }

    private String dotJoin(String root, String name) {
        if (StringUtils.isEmpty(root)) {
            return name;
        }
        else {
            return Joiners.DOT.join(root, name);
        }
    }

}
