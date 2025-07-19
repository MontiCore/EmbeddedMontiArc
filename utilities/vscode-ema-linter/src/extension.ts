import { ExtensionContext } from 'vscode';
import { join } from 'path';
import { getLogger, configure } from 'log4js';
import { allDependenciesAvailable, LspExtensionManager } from '@monticore/monticore-vscode-commons';

let lspExtensionManager: LspExtensionManager | null = null;

export function activate(context: ExtensionContext) {
	setupLog(context.extensionPath, "ema-lint.log");
	if(allDependenciesAvailable(["java", "mvn"])){
		lspExtensionManager = LspExtensionManager.fromSettingsFile(context, "emalinter.autoupdate", join(context.extensionPath, "settings", "global"));
		lspExtensionManager.activate();
	}else{
		getLogger().fatal("Aborting: Missing executables!");
	}
}

function setupLog(extensionPath: string, logName: string) {
	const logFileName = extensionPath + '/logs/' + logName;
	configure({
		appenders: {
			out: { type: 'console' },
			app: { type: 'file', filename: logFileName, maxLogSize: 1000000, backups: 1 }
		},
		categories: {
			default: { appenders: ['out', 'app'], level: 'trace' }
		}
	});
	getLogger().debug("Logger created! Will write to " + logFileName);
}

export function deactivate() {
	if(lspExtensionManager){
		lspExtensionManager.deactivate();
	}
}
