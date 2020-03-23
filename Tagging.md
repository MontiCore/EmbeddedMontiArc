# Tagging
The *Tagging* language consist of 3 grammars:
- **Tagging**: basic infrastructure and classes to provide tagging
 functionality 
- **TagSchema**: definition of the available tags
- **TagValue**: defines possible value types for the tags

The purpose of tagging is the enrichment of other language models with
 additional information without having to put them in the same model.

# Tagging
The main pupose of this language infrastructure for the whole tagging
 functionality.

The grammar file is [`de.monticore.lang.Tagging`][TaggingGrammar].

## Handwritten Extensions
## SymbolTable
- An additional
 [`de.monticore.lang._symboltable.TaggingResolver`][TaggingResolver] is
 provided to be able to connect the symbol tables of the tagging language and
 the respective other language

## Functionality
### Generator
A generator infrastructure is provided in
 [`de.monticore.lang.tagging.generator`][generator] and can be used with the
 provided [`de.monticore.lang.tagging.groovy`][groovy] script.

# TagSchema
The TagSchema defines available TagTypes.

The grammar file is [`de.monticore.lang.TagSchema`][TagSchema].

## Handwritten Extensions
## AST
- There are simplifications for
 [`de.monticore.lang.tagging.tagschema._ast.ASTComplexTagType`][ASTComplexTagType]
 and
 [`de.monticore.lang.tagging.tagschema._ast.ASTEnumeratedTagType`][ASTEnumeratedTagType]


# TagValue
The TagValue defines available types for the values of the tags.

The grammar file is [`de.monticore.lang.TagValue`][TagValue].

[TaggingGrammar]: https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/grammars/de/monticore/lang/Tagging.mc4
[TaggingResolver]: https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/java/de/monticore/lang/tagging/_symboltable/TaggingResolver.java
[generator]: https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/tree/master/src%2Fmain%2Fjava%2Fde%2Fmonticore%2Flang%2Ftagging%2Fgenerator
[groovy]: https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/resources/de/monticore/lang/tagging.groovy
[TagSchema]: https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/grammars/de/monticore/lang/TagSchema.mc4
[ASTComplexTagType]: https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/java/de/monticore/lang/tagschema/_ast/ASTComplexTagType.java
[ASTEnumeratedTagType]: https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/java/de/monticore/lang/tagschema/_ast/ASTEnumeratedTagType.java
[TagValue]: https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/Tagging/-/blob/master/src/main/grammars/de/monticore/lang/TagValue.mc4
