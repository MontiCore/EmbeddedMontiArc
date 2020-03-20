# Tagging
The *Tagging* language consist of 3 grammars:
- **Tagging**: basic infrastructure and classes to provide tagging functionality 
- **TagSchema**: definition of the available tags
- **TagValue**: defines possible value types for the tags

The purpose of tagging is the enrichment of other language models with additional information without having to put them in the same model.

# Tagging
The main pupose of this language infrastructure for the whole tagging functionality.

The grammar file is [`de.monticore.lang.Tagging`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/grammars/de/monticore/lang/Tagging.mc4).

## Handwritten Extensions
## SymbolTable
- An additional [`de.monticore.lang._symboltable.TaggingResolver`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/java/de/monticore/lang/tagging/_symboltable/TaggingResolver.java) is provided to be able to connect the symbol tables of the tagging language and the respective other language

## Functionality
### Generator
A generator infrastructure is provided in [`de.monticore.lang.tagging.generator`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/tree/master/src%2Fmain%2Fjava%2Fde%2Fmonticore%2Flang%2Ftagging%2Fgenerator) and can be used with the provided [`de.monticore.lang.tagging.groovy`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/resources/de/monticore/lang/tagging.groovy) script


# TagSchema
The TagSchema defines available TagTypes.

The grammar file is [`de.monticore.lang.TagSchema`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/grammars/de/monticore/lang/TagSchema.mc4).

## Handwritten Extensions
## AST
- There are simplifications for [`de.monticore.lang.tagging.tagschema._ast.ASTComplexTagType`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/java/de/monticore/lang/tagschema/_ast/ASTComplexTagType.java) and [`de.monticore.lang.tagging.tagschema._ast.ASTEnumeratedTagType`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/java/de/monticore/lang/tagschema/_ast/ASTEnumeratedTagType.java)


# TagValue
The TagValue defines available types for the values of the tags.

The grammar file is [`de.monticore.lang.TagValue`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/grammars/de/monticore/lang/TagValue.mc4).
