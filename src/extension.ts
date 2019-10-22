import * as vscode from 'vscode';
import { join } from 'path';
import * as log4js from 'log4js';
import { getLogger } from 'log4js';
import { MavenLanguageClient, MavenLanguageClientOptions } from './mavenLanguageClient';
import { MavenUpdater } from './mavenUpdater';
import { MavenUpdateManager } from './mavenUpdateManager';
import { LanguageServerManager } from './languageServerManager';
import { spawnSync } from 'child_process';

let updateManager: MavenUpdateManager = new MavenUpdateManager("emalinter.autoupdate");
let languageServerManager = new LanguageServerManager();

export function activate(context: vscode.ExtensionContext) {
	let globalOptions = require(join(context.extensionPath, "settings", "global"));

	setupLog(context.extensionPath, globalOptions.logName);
	getLogger().debug("activate");

	if(dependenciesAvailable()){
		for (let clientOptionsPath of globalOptions.clientOptions) {
			let clientOptions: MavenLanguageClientOptions = require(join(context.extensionPath, clientOptionsPath));
			languageServerManager.addClient(new MavenLanguageClient(context, clientOptions));
			updateManager.addUpdater(new MavenUpdater(clientOptions.languageName, join(context.extensionPath, clientOptions.pomRoot), clientOptions.relativeMvnSettingsPath));
		}
	
		vscode.window.onDidChangeVisibleTextEditors(editors => {
			for (let editor of editors) {
				languageServerManager.activateClient(editor.document.languageId);
			}
		});
	
		// activate linters for files that are visible at activation time
		for (let editor of vscode.window.visibleTextEditors) {
			languageServerManager.activateClient(editor.document.languageId);
		}
	
		languageServerManager.activateWellnessCheck(10 * 1000);
		setTimeout(() => updateManager.checkForUpdates(), 30 * 1000);
	}else{
		getLogger().fatal("Aborting: Missing executables!");
	}
}

function dependenciesAvailable():boolean {
	let checkCommand: string;
	if (process.platform === "win32") {
		checkCommand = "where";
	} else {
		checkCommand = "whereis";
	}

	let res = true;

	if (spawnSync(checkCommand + " mvn").status !== 0) {
		const errorMsg = "Can not find mvn in PATH. Is Maven installed?";
		getLogger().error(errorMsg);
		vscode.window.showErrorMessage(errorMsg);
		res = false;
	}

	if (spawnSync(checkCommand + " java").status !== 0) {
		const errorMsg = "Can not find java in PATH. Is Java installed?";
		getLogger().error(errorMsg);
		vscode.window.showErrorMessage(errorMsg);
		res = false;
	}

	return res;
}

function setupLog(extensionPath: string, logName: string) {
	const logFileName = extensionPath + '/logs/' + logName;
	log4js.configure({
		appenders: {
			out: { type: 'console' },
			app: { type: 'file', filename: logFileName, maxLogSize: 1000000, backups: 1 }
		},
		categories: {
			default: { appenders: ['out', 'app'], level: 'debug' }
		}
	});
	getLogger().debug("Logger created! Will write to " + logFileName);
}


export function deactivate() {
	languageServerManager.deactivateAll();
}
