/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { QueryHandler } from "./query-controller";
import { RecursivePartial  } from '@theia/core/lib/common';
import URI from "@elysium/core/lib/common/uri";
import { EditorManager, Range } from "@theia/editor/lib/browser";
import { FileUri } from "@theia/core/lib/node/file-uri";
import { FrontendApplicationStateService } from "@theia/core/lib/browser/frontend-application-state";

/**
 * `QueryHandler` which handles the opening of workspace files.
 */
@injectable()
export class OpenFileQueryHandler implements QueryHandler {
    @inject(EditorManager) protected readonly editorManager: EditorManager;
    @inject(FrontendApplicationStateService) protected readonly stateService: FrontendApplicationStateService;

    public canHandle(uri: URI): number {
        return uri.hasQueryParam("openFile") ? 100 : 0;
    }

    public isChainable(): boolean {
        return true;
    }

    public async handle(uri: URI): Promise<void> {
        const root = uri.getQueryParam("root");
        const openFile = uri.getQueryParam("openFile");
        const entries = openFile!.split(',');

        if (root) return this.handleEntries(root, entries);
        else console.warn("[OpenFile]: A root is required in the url.");
    }

    protected convertEntry(entry: string): { path: string, selection?: RecursivePartial<Range> } {
        const parts = entry.split(':');
        const line = parseInt(parts[1], 10) || 0;
        const character = parseInt(parts[2], 10) || 0;
        const position = { line, character };
        const selection = { start: position, end: position };

        return { path: parts[0], selection };
    }

    protected async handleEntries(root: string, entries: string[]): Promise<void> {
        const rootURI = FileUri.create(root);

        await this.stateService.reachedState("ready");

        for (const entry of entries) {
            const { path, selection } = this.convertEntry(entry);

            const pathURI = rootURI.resolve(path);
            const options = { selection };

            await this.editorManager.open(pathURI, options);
        }
    }
}
