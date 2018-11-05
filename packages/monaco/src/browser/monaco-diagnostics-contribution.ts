/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { FrontendApplicationContribution, FrontendApplication } from "@theia/core/lib/browser";
import { EditorManager, EditorWidget } from "@theia/editor/lib/browser";
import { DisposableCollection } from "@theia/core/lib/common";
import { LanguageWorker } from "@elysium/languages/lib/common";
import { MonacoEditor } from "@theia/monaco/lib/browser/monaco-editor";
import { MonacoLanguages } from "@theia/monaco/lib/browser/monaco-languages";
import { DiagnosticCollection, Diagnostic } from "@theia/languages/lib/browser";
import { Cloud9Converter } from "@elysium/languages/lib/browser";

@injectable()
export class MonacoDiagnosticsContribution implements FrontendApplicationContribution {
    @inject(EditorManager) protected readonly editorManager: EditorManager;
    @inject(LanguageWorker) protected readonly worker: LanguageWorker;
    @inject(MonacoLanguages) protected readonly languages: MonacoLanguages;
    @inject(Cloud9Converter) protected readonly converter: Cloud9Converter;

    protected readonly toDispose = new DisposableCollection();

    protected diagnostics: DiagnosticCollection;
    protected timer: number;

    public onStart(app: FrontendApplication) {
        this.diagnostics = this.languages.createDiagnosticCollection("Syntax Errors");

        this.updateDiagnostics();
        this.editorManager.onCurrentEditorChanged(editor => this.updateDiagnosticsForEditor(editor));
    }

    protected updateDiagnostics(): void {
        const editor = this.editorManager.currentEditor;

        if (editor) this.updateDiagnosticsForEditor(editor);
    }

    protected async updateDiagnosticsForEditor(editor: EditorWidget | undefined): Promise<void> {
        if (editor) {
            const model = this.getModel(editor);
            const uri = model.uri.toString();
            const diagnostics = await this.computeDiagnostics(model);

            this.publish(uri, diagnostics);
        }
    }

    protected getModel(editor: EditorWidget): monaco.editor.IModel {
        const monacoEditor = MonacoEditor.get(editor);
        const model = monacoEditor!.getControl().getModel();

        this.toDispose.dispose();
        this.toDispose.push(model.onDidChangeContent(event => {
            this.scheduleDiagnostics(model);
        }));

        return model;
    }

    protected scheduleDiagnostics(model: monaco.editor.IModel): void {
        window.clearTimeout(this.timer);

        this.timer = window.setTimeout(async () => {
            const uri = model.uri.toString();
            const diagnostics = await this.computeDiagnostics(model);

            this.publish(uri, diagnostics);
        }, 200);
    }

    protected async computeDiagnostics(model: monaco.editor.IModel): Promise<Diagnostic[]> {
        const id = model.getModeId();
        const docValue = model.getValue();

        await this.worker.parse(id, docValue);

        const analyzes = await this.worker.analyze(id);

        return this.converter.toDiagnostics(analyzes);
    }

    protected publish(uri: string, diagnostics: Diagnostic[]): void {
        this.diagnostics.set(uri, diagnostics);
    }
}
