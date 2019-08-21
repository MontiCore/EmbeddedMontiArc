/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.visitor;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOptionProp;
import de.monticore.lang.monticar.sol.grammars.options._visitor.OptionsVisitor;
import de.monticore.lang.monticar.sol.grammars.options.cocos.ComponentTypeService;
import de.monticore.lang.monticar.sol.runtime.grammar.ast.ASTProperty;
import de.monticore.mcliterals._ast.ASTBooleanLiteral;
import de.monticore.mcliterals._ast.ASTDoubleLiteral;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.Stack;

@Singleton
public class OptionsSerializerImpl implements OptionsSerializer, OptionsVisitor {
    protected final ComponentTypeService service;
    protected final ASTProperty<JSONObject> values;
    protected final ASTProperty<String> keys;
    protected final Stack<ASTOption> options;

    @Inject
    protected OptionsSerializerImpl(ComponentTypeService service) {
        this.service = service;
        this.values = new ASTProperty<>();
        this.keys = new ASTProperty<>();
        this.options = new Stack<>();
    }

    @Override
    public JSONObject serialize(ASTOption node) {
        this.handle(node);
        return this.values.removeFrom(node);
    }

    @Override
    public void visit(ASTOption node) {
        JSONObject values = new JSONObject();
        String type = node.getType();
        JSONObject props = new JSONObject();

        if (this.options.size() > 0)
            this.values.get(this.options.peek()).getJSONObject("props").getJSONArray("elements").put(values);

        values.put("variable", node.getName());
        values.put("type", type);
        values.put("props", props);

        if (this.service.supportsOptions(type)) props.put("elements", new JSONArray());

        this.values.put(node, values);
        this.options.push(node);
    }

    @Override
    public void endVisit(ASTOption node) {
        this.options.pop();

        if (this.options.size() > 1) this.values.removeFrom(node);
    }

    @Override
    public void visit(ASTOptionProp node) {
        this.keys.put(this.options.peek(), node.getName());
    }

    @Override
    public void endVisit(ASTOptionProp node) {
        this.keys.removeFrom(this.options.peek());
    }

    @Override
    public void visit(ASTStringLiteral node) {
        ASTOption parent = this.options.peek();

        this.values.get(parent).getJSONObject("props").put(this.keys.get(parent), node.getValue());
    }

    @Override
    public void visit(ASTDoubleLiteral node) {
        ASTOption parent = this.options.peek();

        this.values.get(parent).getJSONObject("props").put(this.keys.get(parent), node.getValue());
    }

    @Override
    public void visit(ASTBooleanLiteral node) {
        ASTOption parent = this.options.peek();

        this.values.get(parent).getJSONObject("props").put(this.keys.get(parent), node.getValue());
    }
}
