import { ExtensionContext, window } from "vscode";
import { join } from "path";
import { MavenUpdateManager } from "./mavenUpdateManager";
import { LanguageServerManager } from "./languageServerManager";
import { MavenLanguageClientOptions, MavenLanguageClient } from "./mavenLanguageClient";
import { MavenUpdater } from "./mavenUpdater";

export class LspExtensionManager {
    private updateManager: MavenUpdateManager;
    private languageServerManager: LanguageServerManager;
    private context: ExtensionContext;

    constructor(configName: string, context: ExtensionContext) {
        this.context = context;
        this.updateManager = new MavenUpdateManager(configName);
        this.languageServerManager = new LanguageServerManager();
    }

    public static fromSettingsFile(context: ExtensionContext, configName: string, path: string): LspExtensionManager {
        let globalOptions = require(path);
        let options: MavenLanguageClientOptions[] = []
        for (let clientOptionsPath of globalOptions.clientOptions) {
            let clientOptions: MavenLanguageClientOptions = require(join(context.extensionPath, clientOptionsPath));
            options.push(clientOptions);
        }
        return LspExtensionManager.fromSettingsArray(context, configName, options);
    }

    public static fromSettingsArray(context: ExtensionContext, configName: string, clientOptions: MavenLanguageClientOptions[]): LspExtensionManager {
        let res = new LspExtensionManager(configName, context);
        for (let co of clientOptions) {
            res.addClientFromOptions(co);
        }
        return res;
    }

    protected activatePeriodicChecks(): void {
        this.languageServerManager.activateWellnessCheck(10 * 1000);
        setTimeout(() => this.updateManager.checkForUpdates(), 30 * 1000);
    }

    protected addActivationEvents(): void {
        window.onDidChangeVisibleTextEditors(editors => {
            for (let editor of editors) {
                this.languageServerManager.activateClient(editor.document.languageId);
            }
        });

        // activate linters for files that are visible at activation time
        for (let editor of window.visibleTextEditors) {
            this.languageServerManager.activateClient(editor.document.languageId);
        }
    }

    public addClientFromOptions(clientOptions: MavenLanguageClientOptions): void {
        this.languageServerManager.addClient(new MavenLanguageClient(this.context, clientOptions));
        this.updateManager.addUpdater(new MavenUpdater(clientOptions.languages, join(this.context.extensionPath, clientOptions.pomRoot), clientOptions.relativeMvnSettingsPath, clientOptions.useDocker, clientOptions.dockerImage));
    }

    public activate(): void{
        this.addActivationEvents();
        this.activatePeriodicChecks();
    }

    public deactivate(): void {
        this.languageServerManager.deactivateAll();
    }
}