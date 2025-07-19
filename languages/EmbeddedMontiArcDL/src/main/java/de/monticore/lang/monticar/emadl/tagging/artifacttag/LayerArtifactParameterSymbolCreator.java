/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.tagging.artifacttag;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.tagging._ast.ASTNameScope;
import de.monticore.lang.tagging._ast.ASTTag;
import de.monticore.lang.tagging._ast.ASTTagBody;
import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.SymbolKind;
import de.monticore.types.types._ast.ASTQualifiedName;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;

import java.io.File;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class LayerArtifactParameterSymbolCreator implements TagSymbolCreator {

    protected final String regexExpression =  "\\s*\\{\\s*artifact\\s*=\\s*(.+):(.+):(.+)\\s*,\\s*id\\s*=\\s*(.*)\\s*\\}\\s*";
    protected final Pattern pattern = Pattern.compile(regexExpression, Pattern.MULTILINE);

    @Override
    public void create(ASTTaggingUnit unit, TaggingResolver tagging) {
        boolean hasLayerArtifactParameterTag =
            unit.getQualifiedNameList().stream()
                .map(ASTQualifiedName::toString)
                .anyMatch(n -> n.endsWith("LayerArtifactParameterTagSchema"));

        if (!hasLayerArtifactParameterTag) {
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
            addTag(tag, tagging, root, EMADynamicComponentInstantiationSymbol.KIND);
            addTag(tag, tagging, root, EMAComponentSymbol.KIND);
        }
    }

    protected void addTag(ASTTag tag, TaggingResolver tagging, String root, SymbolKind kind) {
        tag.getTagElementList().stream()
            .filter(tagElement -> tagElement.getName().equals("LayerArtifactParameter"))
            .forEachOrdered(tagElement ->  {
                Matcher matcher = matchRegexPattern(tagElement.getTagValue());
                tag.getScopeList().stream()
                    .filter(scope -> scope.getScopeKind().equals("NameScope"))
                    .map(scope -> (ASTNameScope) scope)
                    .map(scope ->
                            tagging.resolve(dotJoin(root, scope.getQualifiedNameString()), kind)
                    )
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .forEachOrdered(scope -> {
                        String artifact = String.format("%s%s%s%s%s", matcher.group(1).replace('.', File.separatorChar),
                            File.separator, matcher.group(2), File.separator, matcher.group(3));
                        String jar = String.format("%s-%s-pretrained", matcher.group(2), matcher.group(3));
                        tagging.addTag(scope, new LayerArtifactParameterSymbol(artifact, jar, matcher.group(4))); }
                    );
            });
    }

    protected Matcher matchRegexPattern(String regex) {
        Matcher matcher = pattern.matcher(regex);
        if (matcher.matches()) {
            return matcher;
        }
        else {
            Log.error(
                String.format(
                        "'%s' does not match the specified regex pattern '%s'",
                        regex, "{artifact = {groupId:artifactId:version}, id = {tagId}}"
                )
            );
            return null;
        }

    }

    protected String dotJoin(String root, String name) {
        if (StringUtils.isEmpty(root)) {
            return name;
        }
        else {
            return Joiners.DOT.join(root, name);
        }
    }

}
