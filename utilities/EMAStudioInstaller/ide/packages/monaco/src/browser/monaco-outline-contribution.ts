/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MonacoOutlineContribution as BaseMonacoOutlineContribution } from "@theia/monaco/lib/browser/monaco-outline-contribution";
import { OutlineItem } from "@elysium/languages/lib/common";
import { LanguageWorker } from "@elysium/languages/lib/common";
import { Cloud9Converter } from "@elysium/languages/lib/browser";
import { MonacoEditor } from "@theia/monaco/lib/browser/monaco-editor";

@injectable()
export class MonacoOutlineContribution extends BaseMonacoOutlineContribution {
    @inject(LanguageWorker) protected readonly worker: LanguageWorker;
    @inject(Cloud9Converter) protected readonly converter: Cloud9Converter;

    protected async updateOutline(): Promise<void> {
        const editor = this.editorManager.currentEditor;

        if (editor) {
            const token = this.tokenSource.token;
            const model = MonacoEditor.get(editor)!.getControl().getModel();
            const uri = model.uri.toString();
            const roots = await this.createRoots(model, token);

            if (roots.length > 0) this.outlineViewService.publish(roots);
            else this.publishFromOutlineItems(uri, await this.computeOutlineItems(model));
        } else {
            this.outlineViewService.publish([]);
        }
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
