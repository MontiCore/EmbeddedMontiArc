import { SpawnSyncReturns, spawnSync, SpawnSyncOptionsWithStringEncoding } from "child_process";
import { getLogger } from "log4js";
import { window } from "vscode"
import { spawn, SpawnOptions} from 'child_process';


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

export function spawnMavenExecChildProcess(mavenPath:string, programArgs: string[], additionalMavenArgs?: string[]){
    let args:string[] = convertProgramArgs(programArgs);
    
    if(additionalMavenArgs){
        args = args.concat(additionalMavenArgs);
    }

    return spawnMavenChildProcess(mavenPath, args);
}

function convertProgramArgs(programArgs: string[]): string[] {
    return ["exec:java", '-e', '-Dexec.args="' + programArgs.join(" ") + '"'];
}

export function spawnMavenChildProcess(mavenPath:string, args: string[]){
    
    let spawnOptions: SpawnOptions = {
        cwd: mavenPath,
        env: process.env,
        stdio: "pipe",
        shell: true
    };

    return spawn("mvn", args, spawnOptions);
}

export function spawnDockerMavenProcess(image: string ,mavenPath:string, args: string[], additionalDockerArgs?:string[]){
    let spawnOptions: SpawnOptions = {
        cwd: mavenPath,
        env: process.env,
        stdio: "pipe",
        shell: true
    };

    let allArgs = [];
    allArgs.push("run");
    allArgs.push("-v");
    allArgs.push(mavenPath + ":/mvn");
    if(additionalDockerArgs){
        for(let ad of additionalDockerArgs){
            allArgs.push(ad);
        }
    }
    allArgs.push(image);
    allArgs.push("mvn");
    for(let a of args){
        allArgs.push(a);
    }
    allArgs.push("-f")
    allArgs.push("/mvn/pom.xml")    

    getLogger().trace("docker " + allArgs.join(" "));

    return spawn("docker", allArgs, spawnOptions);
}

export function spawnDockerMavenExecChildProcess(image: string, mavenPath:string, programArgs: string[], additionalDockerArgs?:string[], additionalMavenArgs?: string[]){
    let args:string[] = convertProgramArgs(programArgs);
    
    if(additionalMavenArgs){
        args = args.concat(additionalMavenArgs);
    }

    return spawnDockerMavenProcess(image, mavenPath, args, additionalDockerArgs);
}