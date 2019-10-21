import * as vscode from 'vscode';
import { join } from 'path';
import * as log4js from 'log4js';
import { getLogger } from 'log4js';
import { MavenLanguageClient, MavenLanguageClientOptions } from './mavenLanguageClient';
import { MavenUpdater } from './mavenUpdater';

let clients: MavenLanguageClient[] = [];
let mavenUpdaters: MavenUpdater[] = [];
let activeClients: MavenLanguageClient[] = [];
let reconnects: number = 0;
let timeout: NodeJS.Timeout;

export function activate(context: vscode.ExtensionContext) {
	let globalOptions = require(join(context.extensionPath, "settings", "global"));

	setupLog(context.extensionPath, globalOptions.logName);
	getLogger().debug("activate");

	for (let clientOptionsPath of globalOptions.clientOptions) {
		let clientOptions: MavenLanguageClientOptions = require(join(context.extensionPath, clientOptionsPath));
		clients.push(new MavenLanguageClient(context, clientOptions));
		mavenUpdaters.push(new MavenUpdater(clientOptions.languageName, join(context.extensionPath, clientOptions.pomRoot), clientOptions.relativeMvnSettingsPath));
	}

	timeout = setInterval(wellnessCheck, 10 * 1000);

	vscode.window.onDidChangeVisibleTextEditors(editors => {
		for (let editor of vscode.window.visibleTextEditors) {
			activateClient(editor.document.languageId);
		}
	});

	// activate linters for files that are visible at activation time
	for (let editor of vscode.window.visibleTextEditors) {
		activateClient(editor.document.languageId);
	}

	setTimeout(() => checkForUpdates(), 30 * 1000);
}


async function getAllUpdateables(): Promise<MavenUpdater[]> {
	let result: MavenUpdater[] = [];
	for (let u of mavenUpdaters) {
		let tmp = await u.isUpdateAvailable();
		if (tmp !== null) {
			result.push(tmp);
		}
	}

	return Promise.resolve(result);
}

function checkForUpdates() {
	let valuesCache: MavenUpdater[] = [];
	getAllUpdateables()
		.then((values) => {
			valuesCache = values;
			if (values.length > 0) {
				let config = vscode.workspace.getConfiguration("emalinter").get("autoupdate");
				if (config === "Always") {
					return true;
				} else if (config === "Never") {
					return false;
				} else {
					return askUpdate();
				}
			}
		})
		.then((update) => {
			let res: Promise<number>[] = [];
			if (update) {
				for (let v of valuesCache) {
					res.push(v.doUpdate());
				}
			}
			return Promise.all(res);
		})
		.then((values) => {
			if (values.length > 0) {
				let sum = values.reduce((acc, cur) => acc + cur);
				if (sum === 0) {
					vscode.window.showInformationMessage("Update was successful!", "Restart now", "Later")
						.then((res) => {
							if (res === "Restart now") {
								vscode.commands.executeCommand("workbench.action.reloadWindow");
							}
						});
				} else {
					vscode.window.showErrorMessage("There was an error while updating. Check the logs for more information.");
				}
			}
		})
		.catch();
}

async function askUpdate(): Promise<boolean> {
	let update = false;
	await vscode.window.showInformationMessage("Updates are available for EmbeddedMontiArc Languages linters! Do you want to update?", "Yes", "Always", "No", "Never")
		.then((res) => {
			if (res === "Yes") {
				update = true;
			}
			if (res === "Always") {
				vscode.workspace.getConfiguration("emalinter").update("autoupdate", "Always", vscode.ConfigurationTarget.Global).then(() => getLogger().debug("Updated preferences to 'Always'"));
				update = true;
			}
			if (res === "Never") {
				vscode.workspace.getConfiguration("emalinter").update("autoupdate", "Never", vscode.ConfigurationTarget.Global).then(() => getLogger().debug("Updated preferences to 'Never'"));
			}
		});
	return update;
}

function activateClient(languageId: string) {
	for (let client of clients) {
		if (client.getLanguageId() === languageId) {
			// give clients time before retrying to connect
			timeout.refresh();

			if (!activeClients.includes(client)) {
				getLogger().debug("Starting client for " + languageId);
				activeClients.push(client);
				client.connect();
			} else {
				getLogger().trace("Client for " + languageId + " already exists! Skipping");
			}
		}
	}
}

function wellnessCheck() {
	getLogger().trace("Running wellness check on " + activeClients.length + " active of " + clients.length + " total clients");
	let reconnectFlag = false;
	for (let client of activeClients) {
		if (!client.isProcessActive()) {
			getLogger().warn(client.getLanguageId() + ": has no active process!");
			reconnectFlag = true;
			if (reconnects < 3) {
				getLogger().info(client.getLanguageId() + ": trying to reconnect");
				client.stop();
				getLogger().debug("Reconnecting");
				client.connect();
			} else {
				getLogger().warn(client.getLanguageId() + ": max number of reconnects reached!");
			}
		}
	}

	if (reconnectFlag) {
		reconnects++;
	} else {
		reconnects = 0;
	}
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
	for (let client of clients) {
		client.stop();
	}
}
