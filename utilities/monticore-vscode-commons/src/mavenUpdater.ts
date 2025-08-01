import { Logger, getLogger } from 'log4js';
import { ChunkBuffer } from './chunkBuffer';
import { spawnMavenChildProcess, spawnDockerMavenProcess, spawnDockerMavenExecChildProcess } from './utils';

export class MavenUpdater{
    private pomPath: string;
    private relativeMvnSettingsPath: string | null;
    private logger: Logger;
    private ids: string[];
    private useDocker?:boolean;
    private dockerImage?:string

    constructor(ids: string[] ,pomPath: string, relativeMvnSettingsPath: string | null = null, useDocker?:boolean, dockerImage?:string){
        this.ids = ids;
        this.pomPath = pomPath;
        this.relativeMvnSettingsPath = relativeMvnSettingsPath;
        this.logger = getLogger("Updater - " + ids);
        this.useDocker = useDocker;
        this.dockerImage = dockerImage;
    }

	doUpdate(): Promise<number> {

        return new Promise<number>((resolve, reject) => {
            let buffer = new ChunkBuffer(1000);
            this.logger.trace("maven path: " + this.pomPath);
            
            const args: string[] = [
                "versions:update-properties",
                "-DallowSnapshots=true"
            ];

            if(this.relativeMvnSettingsPath){
                args.push("-s", this.relativeMvnSettingsPath);
            }
    
            this.logger.debug("Starting update");
            let p;
            if(this.useDocker && this.dockerImage){
                p = spawnDockerMavenProcess(this.dockerImage, this.pomPath, args);
            }else{
                p = spawnMavenChildProcess(this.pomPath, args);
            }

            p.on("error", (err) =>{
                this.logger.error("Error while updating: " + err);
                return reject(err);
            });

            p.on("exit", (code, signal) => {
                if(code === 0){
                    this.logger.info("Update successful");
                }else{
                    this.logger.error("Error while updating: Process exited with code " + code + " from signal " + signal);
                    this.logger.debug(buffer.getText());
                }
                return resolve(code !== null ? code : 1);
            });
            if(p.stdout !== null){
                p.stdout.on("data", (data) => buffer.enq(data));
            }else{
                this.logger.error("p.stdout is null!");
            }
        });
    }

    isUpdateAvailable(): Promise<MavenUpdater | null>{

        this.logger.trace("maven path: " + this.pomPath);
        const args: string[] = [
            "versions:display-dependency-updates",
            '-U',
            "-DallowSnapshots=true"
        ];
        if (this.relativeMvnSettingsPath) {
            args.push("-s", this.relativeMvnSettingsPath);
        }

        return new Promise((resolve, reject) => {
            let circularBuffer = new ChunkBuffer(1000);
            this.logger.debug("spawning maven process!");
            let updateProcess;
            
            if(this.useDocker && this.dockerImage){
                updateProcess = spawnDockerMavenExecChildProcess(this.dockerImage, this.pomPath, args);
            }else{
                updateProcess = spawnMavenChildProcess(this.pomPath, args);
            }

            updateProcess.on("error", (err) => {
                this.logger.error("Maven error: " + err);
                reject(null);
            });

            updateProcess.on("exit" ,(code, signal) => {
                this.logger.debug("Maven process exited with code " + code + " and signal " + signal);
                resolve(null);
            });
            if(updateProcess.stdout !== null){
                updateProcess.stdout.on("data" , (data) => {
                    circularBuffer.enq(data);
                    if(circularBuffer.getText().includes("The following dependencies in Dependencies have newer versions")){
                        this.logger.debug("Found updates!");
                        circularBuffer.clear();
                        resolve(this);
                    }
                });
            }else{
                this.logger.error("updateProcess.stdout is null!");
            }
        });
    }
}