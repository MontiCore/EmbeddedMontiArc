/* (c) https://github.com/MontiCore/monticore */
import { MavenUpdater } from "./mavenUpdater";
import { Logger, getLogger } from "log4js";
import * as vscode from 'vscode';

export class MavenUpdateManager{
    private mavenUpdaters: MavenUpdater[] = [];
    private logger: Logger = getLogger("UpdateManager");
    private configName: string;

    constructor(configName:string){
        this.configName = configName;
    }

    addUpdater(updater: MavenUpdater){
        this.mavenUpdaters.push(updater);
    }

    checkForUpdates() {
        let valuesCache: MavenUpdater[] = [];
        this.getAllUpdateables()
            .then((values) => {
                valuesCache = values;
                if (values.length > 0) {
                    let config = vscode.workspace.getConfiguration().get(this.configName);
                    if (config === "Always") {
                        return true;
                    } else if (config === "Never") {
                        return false;
                    } else {
                        return this.askUpdate();
                    }
                }else{
                    return false;
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

    private async getAllUpdateables(): Promise<MavenUpdater[]> {
        let result: MavenUpdater[] = [];
        for (let u of this.mavenUpdaters) {
            let tmp = await u.isUpdateAvailable();
            if (tmp !== null) {
                result.push(tmp);
            }
        }
    
        return Promise.resolve(result);
    }
    
    private async askUpdate(): Promise<boolean> {
        let update = false;
        await vscode.window.showInformationMessage("Updates are available for EmbeddedMontiArc Languages linters! Do you want to update?", "Yes", "Always", "No", "Never")
            .then((res) => {
                if (res === "Yes") {
                    update = true;
                }
                if (res === "Always") {
                    vscode.workspace.getConfiguration().update(this.configName, "Always", vscode.ConfigurationTarget.Global).then(() => this.logger.debug("Updated preferences to 'Always'"));
                    update = true;
                }
                if (res === "Never") {
                    vscode.workspace.getConfiguration().update(this.configName, "Never", vscode.ConfigurationTarget.Global).then(() => this.logger.debug("Updated preferences to 'Never'"));
                }
            });
        return update;
    }
}
