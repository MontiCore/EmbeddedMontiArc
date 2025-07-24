/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { CommandContribution, CommandRegistry, Command } from "@theia/core/lib/common";
import { TestingConditions } from "./testing-conditions";
import { EditorManager } from "@theia/editor/lib/browser";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { TESTING_SCRIPT, TESTING_ALL_SCRIPT } from "../common";
import { MonacoLanguages } from "@theia/monaco/lib/browser/monaco-languages";
import { DiagnosticCollection, DiagnosticSeverity, Diagnostic } from "@theia/languages/lib/browser";
import { PathsServer } from "@emastudio/paths/lib/common";
import { TESTING_PATH_ID } from "../common";
import { FileStat, FileSystem } from "@theia/filesystem/lib/common";

import URI from "@theia/core/lib/common/uri";

export const TESTING_ICON_CLASS: string = "emastudio-testing-icon";

export namespace TestingCommands {
    export const TEST: Command = {
        id: "emastudio.testing.test",
        label: "Test active Stream",
        iconClass: TESTING_ICON_CLASS
    };

    export const TEST_ALL: Command = {
        id: "emastudio.testing.test.all",
        label: "Test Workspace",
        iconClass: TESTING_ICON_CLASS
    };
}

@injectable()
export class TestingCommandContribution implements CommandContribution {
    @inject(TestingConditions) protected readonly conditions: TestingConditions;
    @inject(EditorManager) protected readonly editorManager: EditorManager;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService;
    @inject(PathsServer) protected readonly pathsServer: PathsServer;
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    protected readonly diagnostics: DiagnosticCollection;

    public constructor(@inject(MonacoLanguages) languages: MonacoLanguages) {
        this.diagnostics = languages.createDiagnosticCollection("Stream Tests");
    }

    public async registerCommands(registry: CommandRegistry): Promise<void> {
        await Promise.all([
            this.registerCommand(registry),
            this.registerCommandAll(registry)
        ]);
    }

    protected async registerCommand(registry: CommandRegistry): Promise<void> {
        const command = registry.registerCommand(TestingCommands.TEST, {
            execute: this.executeTest.bind(this),
            isEnabled: this.isEnabled.bind(this)
        });

        return this.conditions.check(command);
    }

    protected async registerCommandAll(registry: CommandRegistry): Promise<void> {
        const command = registry.registerCommand(TestingCommands.TEST_ALL, {
            execute: this.executeTests.bind(this)
        });

        return this.conditions.checkAll(command);
    }

    protected isEnabled(): boolean {
        const currentEditor = this.editorManager.currentEditor;
        const uri = currentEditor ? currentEditor.getResourceUri() : undefined;

        return uri !== undefined && uri.toString(true).endsWith(".stream");
    }

    protected async executeTest(): Promise<void> {
        const model = await this.getCurrentModel();
        const process = await this.scriptsService.execute({
            label: "Test",
            plugin: "testing",
            script: TESTING_SCRIPT,
            args: [model]
        });

        if (process) process.onExit(this.onTestProcessExit.bind(this));
    }

    protected async onTestProcessExit(): Promise<void> {
        return this.updateDiagnostics();
    }

    protected async getCurrentModel(): Promise<string> {
        const currentEditor = this.editorManager.currentEditor;
        const uri = currentEditor ? currentEditor.getResourceUri() : undefined;
        const roots = await this.workspaceService.roots;
        const workspace = new URI(roots[0].uri);
        const relativePath = workspace.relative(uri!);
        const qualifiedParts = relativePath!.toString().replace(".stream", '').split('/');
        const length = qualifiedParts.length;
        const modelName = qualifiedParts[length - 1];

        qualifiedParts[length - 1] = modelName.charAt(0).toLowerCase() + modelName.slice(1);

        return qualifiedParts.join('.');
    }

    protected async executeTests(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Test (All)",
            plugin: "testing",
            script: TESTING_ALL_SCRIPT
        });

        if (process) process.onExit(this.onTestsProcessExit.bind(this));
    }

    protected async onTestsProcessExit(): Promise<void> {
        return this.updateDiagnostics();
    }

    protected async getModelFile(model: string): Promise<URI> {
        const roots = await this.workspaceService.roots;
        const workspace = new URI(roots[0].uri);
        const qualifiedParts = model.split('.');
        const length = qualifiedParts.length;
        const modelName = qualifiedParts[length - 1];

        qualifiedParts[length - 1] = modelName.charAt(0).toUpperCase() + modelName.slice(1) + ".stream";

        return workspace.resolve(qualifiedParts.join('/'));
    }

    protected async updateDiagnostics(): Promise<void> {
        const uri = await this.pathsServer.getPath(TESTING_PATH_ID);
        const stat = await this.fileSystem.getFileStat(uri);

        if (stat && stat.children) return this.updateDiagnosticsForTests(stat.children);
    }

    protected async updateDiagnosticsForTests(tests: FileStat[]): Promise<void> {
        for (const test of tests) {
            await this.updateDiagnosticsForTest(test);
        }
    }

    protected async updateDiagnosticsForTest(test: FileStat): Promise<void> {
        const contents = await this.fileSystem.resolveContent(test.uri);
        const content = contents.content.replace(/(?:\r\n|\r|\n)/g, ' ');
        const regex = /FAILED: (.*?) ===/g;
        const matches = regex.exec(content);
        const uri = new URI(test.uri);
        const model = await this.getModelFile(uri.displayName);

        if (matches && matches.length > 1) return this.updateDiagnosticsForMatches(model, matches);
        else this.diagnostics.set(model.toString(), []);
    }

    protected async updateDiagnosticsForMatches(model: URI, matches: RegExpMatchArray): Promise<void> {
        const diagnostics: Diagnostic[] = [];
        const length = matches.length;

        for (let i = 1; i < length; i++) {
            diagnostics.push(this.createDiagnosticForMatch(matches[i]));
        }

        this.diagnostics.set(model.toString(), diagnostics);
    }

    protected createDiagnosticForMatch(message: string): Diagnostic {
        const range = { start: { line: 0, character: 0 }, end: { line: 0, character: 0 } };

        return { message, range, severity: DiagnosticSeverity.Error };
    }
}
