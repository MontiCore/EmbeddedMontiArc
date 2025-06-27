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

public class DatasetArtifactSymbolCreator implements TagSymbolCreator {

  protected final String regexExpression = "\\s*\\{\\s*artifact\\s*=\\s*(.+):(.+):(.+)\\s*,\\s*type\\s*=\\s*(.*)\\s*\\}\\s*";
  protected final Pattern pattern = Pattern.compile(regexExpression, Pattern.MULTILINE);

  @Override
  public void create(ASTTaggingUnit taggingUnit, TaggingResolver resolver) {
    boolean hasLayerArtifactParameterTag =
        taggingUnit.getQualifiedNameList().stream()
            .map(ASTQualifiedName::toString)
            .anyMatch(n -> n.endsWith("DatasetArtifactTagSchema"));

    if (!hasLayerArtifactParameterTag) {
      return;
    }

    final String packageName = Joiners.DOT.join(taggingUnit.getPackageList());
    final ASTTagBody tagBody = taggingUnit.getTagBody();
    final String root =
        (tagBody.getTargetModelOpt().isPresent()) ?
            Joiners.DOT.join(packageName, tagBody.getTargetModelOpt().get()
                .getQualifiedNameString()) :
            packageName;


    for (ASTTag tag : taggingUnit.getTagBody().getTagList()) {
      addTag(tag, resolver, root, EMADynamicComponentInstantiationSymbol.KIND);
      addTag(tag, resolver, root, EMAComponentSymbol.KIND);
    }
  }

  protected void addTag(ASTTag tag, TaggingResolver tagging, String root, SymbolKind kind) {
    tag.getTagElementList().stream()
        .filter(tagElement -> tagElement.getName().equals("DatasetArtifact"))
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
                String groupId = matcher.group(1);
                String artifactId = matcher.group(2);
                String version = matcher.group(3);
                String artifact = String.format("%s%s%s%s%s", groupId.replace('.', File.separatorChar),
                    File.separator, artifactId, File.separator, version);
                String jar = String.format("%s-%s-dataset", artifactId, version);
                tagging.addTag(scope, new DatasetArtifactSymbol(artifact, jar, matcher.group(4), artifactId, groupId, version)); }
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
              regex, "{artifact = {groupId:artifactId:version}}"
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
