/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { AccessContribution } from "./access-controller";
import { FileSystem } from "@theia/filesystem/lib/common";
import { EditorManager } from "@theia/editor/lib/browser";
import { WorkspaceServer } from "@theia/workspace/lib/common";
import { MonacoLanguages } from "@theia/monaco/lib/browser/monaco-languages";
import { FrontendApplicationStateService } from "@theia/core/lib/browser/frontend-application-state";
import { FileUri } from "@theia/core/lib/node/file-uri";

/**
 * `AccessContribution` which enables the internal or external use of the FileSystem.
 */
@injectable()
export class FileSystemAccessContribution implements AccessContribution {
    @inject(FileSystem) public readonly contribution: FileSystem;

    public readonly id: string = "filesystem";
}

/**
 * `AccessContribution` which enables the internal or external use of the EditorManager.
 */
@injectable()
export class EditorManagerAccessContribution implements AccessContribution {
    @inject(EditorManager) public readonly contribution: EditorManager;

    public readonly id: string = "editorManager";
}

/**
 * `AccessContribution` which enables the internal or external creation of URI objects.
 */
@injectable()
export class FileURIAccessContribution implements AccessContribution {
    public readonly contribution: object = FileUri;
    public readonly id: string = "fileURI";
}

/**
 * `AccessContribution` which enables the internal or external access to the `WorkspaceServer`.
 */
@injectable()
export class WorkspaceAccessContribution implements AccessContribution {
    @inject(WorkspaceServer) public readonly contribution: WorkspaceServer;

    public readonly id: string = "workspace";
}

/**
 * `AccessContribution` which enables the internal or external access to `MonacoLanguages`.
 */
@injectable()
export class MonacoLanguagesAccessContribution implements AccessContribution {
    @inject(MonacoLanguages) public readonly contribution: MonacoLanguages;

    public readonly id: string = "monacoLanguages";
}

/**
 * `AccessContribution` which enables the internal or external access to `FrontendApplicationStateService`.
 */
@injectable()
export class FrontendApplicationStateServiceAccessContribution implements AccessContribution {
    @inject(FrontendApplicationStateService) public readonly contribution: FrontendApplicationStateService;

    public readonly id: string = "frontendApplicationStateService";
}
