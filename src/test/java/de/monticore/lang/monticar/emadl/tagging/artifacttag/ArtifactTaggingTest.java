package de.monticore.lang.monticar.emadl.tagging.artifacttag;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.emadl.AbstractTaggingResolverTest;
import de.monticore.lang.monticar.emadl.tagging.dltag.DataPathSymbol;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.Collection;

import static org.junit.Assert.assertEquals;

public class ArtifactTaggingTest extends AbstractTaggingResolverTest {

  private TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources");

  @Test
  public void testValidDatasetArtifactTag() {
    EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>resolve("tagging.artifacttagging.Alexnet", EMAComponentSymbol.KIND)
        .orElse(null);

    Collection<TagSymbol> tags = tagging.getTags(symbol, DatasetArtifactSymbol.KIND);
    assertEquals(1, tags.size());

    DatasetArtifactSymbol datasetArtifactSymbol = (DatasetArtifactSymbol) tags.iterator().next();
    assertEquals("com/emadl/dataset/mnist/2", datasetArtifactSymbol.getArtifact());
    assertEquals("mnist-2-dataset", datasetArtifactSymbol.getJar());
    assertEquals("HDF5", datasetArtifactSymbol.getType());
  }

  @Test
  public void testValidLayerArtifactParameterTag() {
    EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>resolve("tagging.artifacttagging.Parent", EMAComponentSymbol.KIND).get();

    Collection<TagSymbol> tagsA1 = tagging.getTags(symbol.getSpannedScope().getLocalSymbols().get("a1").iterator().next(), DatasetArtifactSymbol.KIND);
    Collection<TagSymbol> tagsA2 = tagging.getTags(symbol.getSpannedScope().getLocalSymbols().get("a2").iterator().next(), DatasetArtifactSymbol.KIND);
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






}
