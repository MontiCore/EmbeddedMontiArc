/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.tagging.artifacttag;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.monticar.emadl.AbstractTaggingResolverTest;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.Collection;

import static org.junit.Assert.assertEquals;

public class ArtifactTaggingTest extends AbstractTaggingResolverTest {

  private TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources");
  private EMAComponentSymbol parentSymbol =
      tagging.<EMAComponentSymbol>resolve("tagging.artifacttagging.Parent", EMAComponentSymbol.KIND).orElse(null);
  private EMAComponentSymbol alexnetSymbol =
      tagging.<EMAComponentSymbol>resolve("tagging.artifacttagging.Alexnet", EMAComponentSymbol.KIND).orElse(null);

  @Test
  public void testValidDatasetArtifactTagForComponent() {
    Collection<TagSymbol> tags = tagging.getTags(alexnetSymbol, DatasetArtifactSymbol.KIND);
    assertEquals(1, tags.size());

    DatasetArtifactSymbol datasetArtifactSymbol = (DatasetArtifactSymbol) tags.iterator().next();
    assertEquals("com/emadl/dataset/mnist/2", datasetArtifactSymbol.getArtifact());
    assertEquals("mnist-2-dataset", datasetArtifactSymbol.getJar());
    assertEquals("HDF5", datasetArtifactSymbol.getType());
  }

  @Test
  public void testValidDatasetArtifactTagForComponentForInstances() {
    Collection<TagSymbol> tagsA1 = tagging.getTags(parentSymbol.getSpannedScope().getLocalSymbols().get("a1").iterator().next(), DatasetArtifactSymbol.KIND);
    Collection<TagSymbol> tagsA2 = tagging.getTags(parentSymbol.getSpannedScope().getLocalSymbols().get("a2").iterator().next(), DatasetArtifactSymbol.KIND);
    assertEquals(1, tagsA1.size());
    assertEquals(1, tagsA2.size());

    DatasetArtifactSymbol tagA1 = (DatasetArtifactSymbol) tagsA1.iterator().next();
    assertEquals("com/emadl/dataset/sst2/40", tagA1.getArtifact());
    assertEquals("sst2-40-dataset", tagA1.getJar());
    assertEquals("LMDB", tagA1.getType());

    DatasetArtifactSymbol tagA2 = (DatasetArtifactSymbol) tagsA2.iterator().next();
    assertEquals("com/monticore/lang/monticar/imdb/2", tagA2.getArtifact());
    assertEquals("imdb-2-dataset", tagA2.getJar());
    assertEquals("HDF5", tagA2.getType());
  }

  @Test
  public void testValidLayerArtifactParameterTagForComponent() {
    Collection<TagSymbol> tags = tagging.getTags(alexnetSymbol, LayerArtifactParameterSymbol.KIND);
    assertEquals(1, tags.size());

    LayerArtifactParameterSymbol layerArtifactParameterSymbol = (LayerArtifactParameterSymbol) tags.iterator().next();
    assertEquals("com/emadl/pretrained-model/bert-small/2", layerArtifactParameterSymbol.getArtifact());
    assertEquals("bert-small-2-pretrained", layerArtifactParameterSymbol.getJar());
    assertEquals("bert-small", layerArtifactParameterSymbol.getId());
  }

  @Test
  public void testValidLayerArtifactParameterTagForComponentForInstances() {
    Collection<TagSymbol> tagsA1 = tagging.getTags(parentSymbol.getSpannedScope().getLocalSymbols().get("a1").iterator().next(), LayerArtifactParameterSymbol.KIND);
    assertEquals(1, tagsA1.size());

    LayerArtifactParameterSymbol layerArtifactParameterSymbolA1 = (LayerArtifactParameterSymbol) tagsA1.iterator().next();
    assertEquals("com/emadl/pretrained-model/bert-large/1", layerArtifactParameterSymbolA1.getArtifact());
    assertEquals("bert-large-1-pretrained", layerArtifactParameterSymbolA1.getJar());
    assertEquals("bert", layerArtifactParameterSymbolA1.getId());
  }


}
