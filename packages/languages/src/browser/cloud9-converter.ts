/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/// <reference types="@theia/monaco/src/typings/monaco" />

import { injectable } from "inversify";
import { Position, AnalyzeItem, OutlineItem } from "../common";
import { Diagnostic, Range, DiagnosticSeverity } from "@theia/languages/lib/browser";
import { MonacoOutlineSymbolInformationNode } from "@theia/monaco/lib/browser/monaco-outline-contribution";

import URI from "@theia/core/lib/common/uri";

import SymbolKind = monaco.languages.SymbolKind;

@injectable()
export class Cloud9Converter {
    public toDiagnosticSeverity(type: string): DiagnosticSeverity {
        switch (type) {
            case "error": return DiagnosticSeverity.Error;
            case "warning": return DiagnosticSeverity.Warning;
            default: return DiagnosticSeverity.Information;
        }
    }

    public toRange(position: Position): Range {
        return {
            start: {
                line: position.sl || 0,
                character: position.sc || 0
            },
            end: {
                line: position.el || 0,
                character: position.ec || 0
            }
        };
    }

    public toDiagnostic(item: AnalyzeItem): Diagnostic {
        return {
            range: this.toRange(item.pos!),
            severity: this.toDiagnosticSeverity(item.type || "warning"),
            message: item.message || ''
        };
    }

    public toDiagnostics(items: AnalyzeItem[]): Diagnostic[] {
        const diagnostics = [];

        for (const item of items) {
            const diagnostic = this.toDiagnostic(item);

            diagnostics.push(diagnostic);
        }

        return diagnostics;
    }

    public toSymbolKind(icon: string): SymbolKind {
        switch (icon) {
            case "method": return SymbolKind.Field;
            case "property": return SymbolKind.Module;
            case "property2": return SymbolKind.Constant;
            case "method2": return SymbolKind.Method;
            case "event": return SymbolKind.Property;
            default: return SymbolKind.Variable;
        }
    }

    public toOutlineNode(item: OutlineItem, uri: string, parent: MonacoOutlineSymbolInformationNode | undefined): MonacoOutlineSymbolInformationNode {
        const kind = this.toSymbolKind(item.icon!);
        const range = this.toRange(item.pos!);
        const node = {
            id: item.name || '',
            iconClass: SymbolKind[kind].toString().toLowerCase(),
            name: item.name || '',
            parent,
            uri: new URI(uri),
            range,
            fullRange: range,
            selected: false,
            expanded: true,
            children: [] as MonacoOutlineSymbolInformationNode[]
        };

        node.children = this.toOutlineNodes(item.items || [], uri, node);

        return node;
    }

    public toOutlineNodes(items: OutlineItem[], uri: string, parent: MonacoOutlineSymbolInformationNode | undefined): MonacoOutlineSymbolInformationNode[] {
        const nodes: MonacoOutlineSymbolInformationNode[] = [];

        for (const item of items) {
            const node = this.toOutlineNode(item, uri, parent);

            nodes.push(node);
        }

        return nodes;
    }
}
