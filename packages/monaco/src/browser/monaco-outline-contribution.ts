/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MonacoOutlineContribution as BaseMonacoOutlineContribution } from "@theia/monaco/lib/browser/monaco-outline-contribution";
import { OutlineItem } from "@elysium/languages/lib/common";
import { OutlineViewService } from "@theia/outline-view/lib/browser/outline-view-service";
import { EditorManager, EditorWidget } from "@theia/editor/lib/browser";
import { LanguageWorker } from "@elysium/languages/lib/common";
import { Cloud9Converter } from "@elysium/languages/lib/browser";
import { MonacoEditor } from "@theia/monaco/lib/browser/monaco-editor";
import { FrontendApplication } from "@theia/core/lib/browser";

@injectable()
export class MonacoOutlineContribution extends BaseMonacoOutlineContribution {
    protected timer: number;

    public constructor(
        @inject(OutlineViewService) protected readonly outlineViewService: OutlineViewService,
        @inject(EditorManager) protected readonly editorManager: EditorManager,
        @inject(LanguageWorker) protected readonly worker: LanguageWorker,
        @inject(Cloud9Converter) protected readonly converter: Cloud9Converter
    ) {
        super(outlineViewService, editorManager);
    }

    public onStart(app: FrontendApplication): void {
        super.onStart(app);
        this.toDispose.push(
            this.editorManager.onCurrentEditorChanged(widget => this.updateOutlineForEditor(widget))
        );
    }

    protected getModel(editor: EditorWidget): monaco.editor.IModel {
        const monacoEditor = MonacoEditor.get(editor);
        const model = monacoEditor!.getControl().getModel();

        this.toDispose.dispose();
        this.toDispose.push(model.onDidChangeContent(event => {
            this.scheduleOutline(model);
        }));

        return model;
    }

    protected async updateOutlineForEditor(editor: EditorWidget | undefined): Promise<void> {
        if (editor) {
            const model = this.getModel(editor);
            const uri = model.uri.toString();
            const entries = await this.computeSymbolInformations(model);

            if (entries.length > 0) this.publish(entries);
            else this.publishFromOutlineItems(uri, await this.computeOutlineItems(model));
        } else {
            this.publish([]);
        }
    }

    protected scheduleOutline(model: monaco.editor.IModel): void {
        window.clearTimeout(this.timer);

        this.timer = window.setTimeout(async () => {
            const uri = model.uri.toString();
            const entries = await this.computeSymbolInformations(model);

            if (entries.length > 0) this.publish(entries);
            else this.publishFromOutlineItems(uri, await this.computeOutlineItems(model));
        }, 200);
    }

    protected async computeOutlineItems(model: monaco.editor.IModel): Promise<OutlineItem[]> {
        const id = model.getModeId();
        const docValue = model.getValue();

        await this.worker.parse(id, docValue);

        return this.worker.outline(id);
    }

    protected publishFromOutlineItems(uri: string, items: OutlineItem[]): void {
        const outlineSymbolInformations = this.converter.toOutlineNodes(items, uri, undefined);

        this.outlineViewService.publish(outlineSymbolInformations);
    }
}
