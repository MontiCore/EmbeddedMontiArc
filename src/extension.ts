/* (c) https://github.com/MontiCore/monticore */
/*---------------------------------------------------------
 * Copyright (C) Microsoft Corporation. All rights reserved.
 *--------------------------------------------------------*/

'use strict';

import * as vscode from 'vscode';
import { WorkspaceFolder, DebugConfiguration, ProviderResult, CancellationToken } from 'vscode';
import { EmamDebugSession } from './stacktraceDebugger/emamDebug';
import * as Net from 'net';
import { StreamTestCodeLensProvider, runCurrentStreamTest, getConfigForCurrentFile } from './codeLens/StreamTestCodeLensProvider';
import * as log4js from 'log4js';
import { SpawnSyncReturns, spawnSync, SpawnSyncOptionsWithStringEncoding } from 'child_process';
import { getLogger } from 'log4js';
import { MavenUpdateManager } from './util/mavenUpdateManager';
import { MavenUpdater } from './util/mavenUpdater';
import { join } from 'path';

/*
 * Set the following compile time flag to true if the
 * debug adapter should run inside the extension host.
 * Please note: the test suite does not (yet) work in this mode.
 */
const EMBED_DEBUG_ADAPTER = true;

export function activate(context: vscode.ExtensionContext) {
	setupLog(context.extensionPath);

	if (dependenciesAvailable) {
		// register a configuration provider for 'emam' debug type
		const provider = new EmamConfigurationProvider();
		context.subscriptions.push(vscode.debug.registerDebugConfigurationProvider('emam', provider));

		let commandDisposable = vscode.commands.registerCommand(
			"streamTest.runCurrentStreamTest",
			runCurrentStreamTest
		);

		// Get a document selector for the CodeLens provider
		// This one is any file that has the language of javascript
		let docSelector = {
			language: "EmbeddedMontiArcMath",
			pattern: "**/*.emam"
		};

		// Register our CodeLens provider
		let codeLensProviderDisposable = vscode.languages.registerCodeLensProvider(
			docSelector,
			new StreamTestCodeLensProvider()
		);

		// Push the command and CodeLens provider to the context so it can be disposed of later
		context.subscriptions.push(commandDisposable);
		context.subscriptions.push(codeLensProviderDisposable);

		if (EMBED_DEBUG_ADAPTER) {
			const factory = new EmamDebugAdapterDescriptorFactory();
			context.subscriptions.push(vscode.debug.registerDebugAdapterDescriptorFactory('emam', factory));
			context.subscriptions.push(factory);
		}

		let updateManager = new MavenUpdateManager("emamdebug.autoupdate");
		updateManager.addUpdater(new MavenUpdater("Generator", join(context.extensionPath, "maven"), "settings.xml"));
		setTimeout(() => updateManager.checkForUpdates(), 10*1000);
	}
}

export function deactivate() {
	// nothing to do
}

function setupLog(extensionPath: string) {
	const logFileName = extensionPath + '/logs/emam-debug.log';
	log4js.configure({
		appenders: {
			out: { type: 'console' },
			app: { type: 'file', filename: logFileName, maxLogSize: 1000000, backups: 1 }
		},
		categories: {
			default: { appenders: ['out', 'app'], level: 'debug' }
		}
	});
	log4js.getLogger().debug("Logger created! Will write to " + logFileName);
}

function spawnExecutableCheck(executable: string): SpawnSyncReturns<string> {
	if (process.platform === "win32") {
		let args = [executable];
		return spawnSync("where", args);
	} else {
		let args = ["-v", executable];
		const spawnOptions: SpawnSyncOptionsWithStringEncoding = { shell: true, encoding: "utf8" };
		return spawnSync("command", args, spawnOptions);
	}
}

function dependenciesAvailable(): boolean {
	let res = true;
	if (spawnExecutableCheck("mvn").status !== 0) {
		const errorMsg = "Can not find mvn in PATH. Is Maven installed?";
		getLogger().error(errorMsg);
		vscode.window.showErrorMessage(errorMsg);
		res = false;
	}

	if (spawnExecutableCheck("java").status !== 0) {
		const errorMsg = "Can not find java in PATH. Is Java installed?";
		getLogger().error(errorMsg);
		vscode.window.showErrorMessage(errorMsg);
		res = false;
	}

	return res;
}


class EmamConfigurationProvider implements vscode.DebugConfigurationProvider {

	/**
	 * Massage a debug configuration just before a debug session is being launched,
	 * e.g. add all missing attributes to the debug configuration.
	 */
	resolveDebugConfiguration(folder: WorkspaceFolder | undefined, config: DebugConfiguration, token?: CancellationToken): ProviderResult<DebugConfiguration> {

		// if launch.json is missing or empty
		if (!config.type && !config.request && !config.name) {
			const editor = vscode.window.activeTextEditor;
			if (editor && editor.document.languageId === 'EmbeddedMontiArcMath') {
				let tmpConfig = getConfigForCurrentFile();
				if (tmpConfig) {
					config = tmpConfig;
				}
			}
		}

		if (!config.program || !config.modelBase) {
			return vscode.window.showInformationMessage("Cannot find a program to debug").then(_ => {
				return undefined;	// abort launch
			});
		}

		return config;
	}
}

class EmamDebugAdapterDescriptorFactory implements vscode.DebugAdapterDescriptorFactory {

	private server?: Net.Server;

	createDebugAdapterDescriptor(session: vscode.DebugSession, executable: vscode.DebugAdapterExecutable | undefined): vscode.ProviderResult<vscode.DebugAdapterDescriptor> {

		if (!this.server) {
			// start listening on a random port
			this.server = Net.createServer(socket => {
				const session = new EmamDebugSession();
				session.setRunAsServer(true);
				session.start(<NodeJS.ReadableStream>socket, socket);
			}).listen(0);
		}

		// make VS Code connect to debug server
		return new vscode.DebugAdapterServer(this.server.address().port);
	}

	dispose() {
		if (this.server) {
			this.server.close();
		}
	}
}
