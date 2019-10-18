import * as vscode from 'vscode';
import { join } from 'path';
import * as log4js from 'log4js';
import { getLogger } from 'log4js';
import { MavenLanguageClient, MavenLanguageClientOptions } from './mavenLanguageClient';

let clients: MavenLanguageClient[] = [];
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
	getLogger().debug("Running wellness check on " + activeClients.length + " active of " + clients.length + " total clients" );
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
