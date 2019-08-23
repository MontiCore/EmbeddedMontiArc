/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montiarc.tagging;

import de.monticore.lang.tagging._ast.ASTScope;
import de.monticore.lang.tagging._ast.ASTTag;
import de.monticore.lang.tagging._ast.ASTTagElement;
import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.tagging._parser.TaggingParser;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Ignore;
import org.junit.Test;

import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Created by MichaelvonWenckstern on 13.06.2016.
 */
public class ParserTagValueTest {

//  @Test
//  public void testUnit() throws Exception {
//    TagValueParser parser = new TagValueParser();
//    assertTrue(parser.parseString_UnitTagValue("150 mW").isPresent());
//  }
//
//  @Test
//  public void testString() throws Exception {
//    TagValueParser parser = new TagValueParser();
//    assertTrue(parser.parseString_StringTagValue("\"MvW\"").isPresent());
//  }
//
//  @Test
//  public void testBoolean() throws Exception {
//    TagValueParser parser = new TagValueParser();
//    assertTrue(parser.parseString_BooleanTagValue("true").isPresent());
//  }

//  @Test
//  public void testTags() throws Exception {
//    TagsParser parser = new TagsParser();
//    ASTTags tags = parser.parse("C:\\Users\\MichaelvonWenckstern\\Documents\\MontiArc4\\01.code\\tagging\\src\\test\\resources\\windTurbine\\PowerConsumptionTags.tag").get();
//    System.out.println(tags.getTagList().size());
//    tags.getTagList().stream().filter(t -> t.getTagValue().isPresent())
//        .forEachOrdered(t -> System.out.println(t.getTagValue().get()));
////    System.out.println(tags.getTagList().get(0).getTagValue().length());
//  }

/*  @Ignore
  @Test
  public void testTagschema2() throws Exception {
    TaggingParser parser = new TaggingParser();
    ASTTaggingUnit tags = parser.parse("C:\\Users\\MichaelvonWenckstern\\Documents\\MontiArc4\\01.code\\tagging\\src\\test\\resources\\windTurbine\\PowerConsumption.tag").get();
    tags.getTagBody().getTagList().forEach(t -> t.getTagElements().forEach(e -> System.out.println(e.getName() + ": " + e.getTagValue().orElse(""))));
//    System.out.println(tags.getTagList().size());
    //    System.out.println(tags.getTagList().get(0).getTagValue().length());
  }*/
}
