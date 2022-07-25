package de.monticore.lang.gdl.visitors;

import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import de.monticore.lang.gdl.GDLMill;
import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameConstruct;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameTupleBuilder;
import de.monticore.lang.gdl._ast.ASTGameTypeDefBuilder;
import de.monticore.lang.gdl._ast.ASTGameTypeMapDef;
import de.monticore.lang.gdl._visitor.GDLTraverser;
import de.monticore.lang.gdl._visitor.GDLVisitor2;

public class ASTTypeInflator implements GDLVisitor2 {
    
    private static final Predicate<ASTGameTuple> TYPEMAPDEF_FILTER = tuple -> 
            tuple.getElement(0) instanceof ASTGameInference
            && tuple.getElement(1) instanceof ASTGameTuple
            && ((ASTGameTuple) tuple.getElement(1)).getElement(0) instanceof ASTGameTypeMapDef;
    
    private static final Function<ASTGameTuple, ASTGameTuple> TYPEDEF_TO_TYPE = tuple -> {
        final ASTGameTypeMapDef typeMapDef = (ASTGameTypeMapDef) ((ASTGameTuple) tuple.getElement(1)).getElement(0);
        final ASTGameConstruct type = (ASTGameConstruct) ((ASTGameTuple) tuple.getElement(1)).getElement(1);
        final ASTGameConstruct typeMap = (ASTGameConstruct) ((ASTGameTuple) tuple.getElement(1)).getElement(2);

        return new ASTGameTupleBuilder()
            .addAllElement(List.of(
                new ASTGameTypeDefBuilder()
                    .set_SourcePositionStart(typeMapDef.get_SourcePositionStart()) 
                    .set_SourcePositionEnd(typeMapDef.get_SourcePositionEnd()) 
                    .build(),
                type,
                typeMap
            ))
            .set_SourcePositionStart(tuple.get_SourcePositionStart())
            .set_SourcePositionEnd(tuple.get_SourcePositionEnd())
            .build();
    };

    private GDLTraverser traverser;

    public ASTTypeInflator() {
        this.traverser = GDLMill.traverser();
        this.traverser.add4GDL(this);
    }

    public GDLTraverser getTraverser() {
        return this.traverser;
    }

    @Override
    public void visit(ASTGame node) {
        List<ASTGameTuple> tupleList =  node.getTuplesList();
        List<ASTGameTuple> inflatedTuples = tupleList.stream()
                .filter(TYPEMAPDEF_FILTER)
                .map(TYPEDEF_TO_TYPE)
                .collect(Collectors.toList());

        node.addAllTuples(inflatedTuples);
    }

}
