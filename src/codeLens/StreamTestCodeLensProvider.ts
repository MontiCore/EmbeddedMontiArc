/* (c) https://github.com/MontiCore/monticore */
import {
	CodeLensProvider,
	CodeLens,
	Command,
	Range,
	TextDocument,
	window,
	workspace,
	DebugConfiguration,
	debug
} from "vscode";
import {
	basename, extname
} from 'path';
import * as log4js from 'log4js';

export class StreamTestCodeLensProvider implements CodeLensProvider {
	async provideCodeLenses(document: TextDocument): Promise<CodeLens[]> {
		let range = new Range(0, 0, 0, 0);
		let c: Command = {
			command: "streamTest.runCurrentStreamTest",
			title: "▶️ Run stream test for current component"
		};
		let codeLens = new CodeLens(range, c);
		return [codeLens];
	}
}

export function runCurrentStreamTest() {
	const launchConfig: EmamDebugConfiguration | null = getConfigForCurrentFile();
	if (launchConfig) {
		//Wait till potentially new launch config is accessible
		setTimeout(() => doRunStreamTest(launchConfig), 50);
	}
}

export function getConfigForCurrentFile(): EmamDebugConfiguration | null {
	let res: EmamDebugConfiguration | null = null;

	var tedit = window.activeTextEditor;
	if (tedit) {
		log4js.getLogger().debug("Cur test:" + tedit.document.fileName);
		var editorContent = tedit.document.getText();

		const fileName = basename(tedit.document.fileName);
		const fileExt = extname(tedit.document.fileName);
		if (fileExt == ".emam") {
			var pack: string | null = getModelPackage(editorContent);
			var compName: string | null = getComponentName(editorContent);

			if (compName && pack) {
				let launchConfig = workspace.getConfiguration('launch', null);
				const configField = 'configurations';
				let configurations = launchConfig[configField];

				let matchingConfigs = configurations
					.filter(c => c.type && c.type == "emam")
					.filter(c => c.program && c.program == pack + "." + compName);
				if (matchingConfigs.length > 0) {
					log4js.getLogger().debug("Found existing config!");
					res = matchingConfigs[0];
				} else {
					res = getDefaultLaunchConfig(fileName, pack, compName);
					configurations.push(res);
					launchConfig.update(configField, configurations, false).then(_ => log4js.getLogger().debug("Added new launch config!"));
				}
			}
		} else {
			log4js.getLogger().debug("Not the right extension: " + fileExt);
		}
	}

	return res;
}

function doRunStreamTest(launchConfig: EmamDebugConfiguration) {
	if(workspace.workspaceFolders){
		debug.startDebugging(workspace.workspaceFolders[0], launchConfig.name).then(_ => log4js.getLogger().debug("Started debugging with command!"));
	}else{
		window.showErrorMessage("Not in a workspace! Please open project as folder!");
	}
}

function getModelPackage(content: string): string | null {
	var res: string | null = null;
	try {
		const packageRegex = /[^package]*package\s+([^;]+);.*/g;
		var matchPack = packageRegex.exec(content);
		if (matchPack) {
			res = matchPack[1].trim();
		}
	} catch (err) {
		log4js.getLogger().debug("Error getting the models package:" + err);
	}
	return res;
}

function getComponentName(content: string): string | null {
	var res: string | null = null;
	const componentRegex = /[^component]*component\s+(\w+).*\{.*/g;
	try {
		var matchComp = componentRegex.exec(content);
		if (matchComp) {
			res = matchComp[1];
			res = res[0].toLowerCase() + res.substring(1, res.length);
		}
	} catch (err) {
		log4js.getLogger().error("Error while parsing component name:" + err);
	}
	return res;
}

export interface EmamDebugConfiguration extends DebugConfiguration {
	type: string;
	request: string;
	name: string;
	program: string;
	stopOnEntry: boolean;
	modelBase: string;
}

function getDefaultLaunchConfig(fileName: string, pack: string, compName: string): EmamDebugConfiguration {
	return {
		"type": "emam",
		"request": "launch",
		"name": "" + fileName,
		"program": pack + "." + compName,
		"stopOnEntry": true,
		"modelBase": "${workspaceFolder}"
	};
}
