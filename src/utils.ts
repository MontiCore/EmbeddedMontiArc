import { SpawnSyncReturns, spawnSync, SpawnSyncOptionsWithStringEncoding } from "child_process";
import { getLogger } from "log4js";
import { window } from "vscode"

export function spawnExecutableCheck(executable:string):SpawnSyncReturns<string> {
	if (process.platform === "win32"){
		let args = [executable];
		return spawnSync("where", args);
	}else{
		let args = ["-v", executable];
		const spawnOptions: SpawnSyncOptionsWithStringEncoding = { shell: true, encoding: "utf8" };
		return spawnSync("command", args, spawnOptions);
	}
}

export function dependencyAvailable(command:string, longName:String|null = null):boolean {
	let res = true;
	if (spawnExecutableCheck(command).status !== 0) {
        const tmpName = longName === null ? command : longName;
		const errorMsg = "Can not find " + command +" in PATH. Is " + tmpName + " installed?";
		getLogger().error(errorMsg);
		window.showErrorMessage(errorMsg);
		res = false;
	}
	return res;
}

export function allDependenciesAvailable(commands:string[]){
    let res = true;
    for(let c of commands){
        res = dependencyAvailable(c) && res;
    }
    return res;
}