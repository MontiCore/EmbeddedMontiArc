/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.ld;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguage;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageNode;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTTemplateDeclaration;
import de.monticore.lang.monticar.sol.grammars.language._visitor.LanguageVisitor;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTPair;
import de.monticore.lang.monticar.sol.grammars.options.visitor.OptionsSerializer;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.runtime.grammar.ast.ASTProperty;
import de.monticore.mcliterals._ast.ASTBooleanLiteral;
import de.monticore.mcliterals._ast.ASTDoubleLiteral;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;
import java.util.Stack;

@Singleton
public class LDExtractorImpl implements LDExtractor, LanguageVisitor {
    protected final NotificationService notifications;
    protected final OptionsSerializer serializer;
    protected final ASTProperty<JSONObject> values;
    protected final ASTProperty<String> cache;
    protected final List<ASTTemplateDeclaration> templates;
    protected final Stack<ASTLanguageNode> nodes;

    @Inject
    protected LDExtractorImpl(NotificationService notifications, OptionsSerializer serializer) {
        this.notifications = notifications;
        this.serializer = serializer;
        this.values = new ASTProperty<>();
        this.cache = new ASTProperty<>();
        this.templates = new ArrayList<>();
        this.nodes = new Stack<>();
    }

    @Override
    public List<ASTTemplateDeclaration> getTemplates(ASTLanguageCompilationUnit node) {
        this.handle(node);

        return this.templates;
    }

    @Override
    public String getIdentifier(ASTTemplateDeclaration node) {
        return this.values.get(node).getString("id");
    }

    @Override
    public String getPath(ASTTemplateDeclaration node) {
        return this.values.get(node).getString("path");
    }

    @Override
    public String getLabel(ASTTemplateDeclaration node) {
        return this.values.get(node).getString("label");
    }

    @Override
    public JSONArray getElements(ASTTemplateDeclaration node) {
        return this.values.get(node).getJSONArray("elements");
    }

    @Override
    public void visit(ASTLanguage node) {
        JSONObject values = new JSONObject();

        values.put("id", node.getName().toLowerCase());
        this.values.put(node, values);
        this.nodes.push(node);
    }

    @Override
    public void endVisit(ASTLanguage node) {
        this.nodes.pop();
    }

    @Override
    public void visit(ASTTemplateDeclaration node) {
        JSONObject values = new JSONObject();
        String id = String.format("%s.%s", this.values.get(this.nodes.peek()).get("id"), node.getName());

        values.put("id", id);
        values.put("elements", new JSONArray());
        this.cache.put(node, "path");
        this.values.put(node, values);
        this.nodes.push(node);
        this.templates.add(node);
    }

    @Override
    public void endVisit(ASTTemplateDeclaration node) {
        this.nodes.pop();
        this.cache.removeFrom(node);
    }

    @Override
    public void visit(ASTPair node) {
        this.cache.put(this.nodes.peek(), node.getKey());
    }

    @Override
    public void visit(ASTStringLiteral node) {
        ASTLanguageNode parent = this.nodes.peek();

        this.values.get(parent).put(this.cache.get(parent), node.getValue());
    }

    @Override
    public void visit(ASTDoubleLiteral node) {
        ASTLanguageNode parent = this.nodes.peek();

        this.values.get(parent).put(this.cache.get(parent), node.getValue());
    }

    @Override
    public void visit(ASTBooleanLiteral node) {
        ASTLanguageNode parent = this.nodes.peek();

        this.values.get(parent).put(this.cache.get(parent), node.getValue());
    }

    @Override
    public void handle(ASTOption node) {
        this.values.get(this.nodes.peek()).getJSONArray("elements").put(this.serializer.serialize(node));
    }
}
