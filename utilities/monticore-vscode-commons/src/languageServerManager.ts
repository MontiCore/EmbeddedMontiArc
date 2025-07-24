import { MavenLanguageClient } from "./mavenLanguageClient";
import { getLogger } from "log4js";

export class LanguageServerManager{

    private clients: MavenLanguageClient[] = [];
    private activeClients: MavenLanguageClient[] = [];
    private reconnects: number = 0;
    private timeout: NodeJS.Timeout | null = null;
    private logger = getLogger("LSManager");

    addClient(client: MavenLanguageClient){
        this.clients.push(client);
    }

    activateClient(languageId: string) {
        for (let client of this.clients) {
            if (client.getLanguageIds().includes(languageId)) {
                // give clients time before retrying to connect
                if(this.timeout !== null){
                    this.timeout.refresh();
                }
    
                if (!this.activeClients.includes(client)) {
                    this.logger.debug("Starting client for " + languageId);
                    this.activeClients.push(client);
                    client.connect();
                } else {
                    this.logger.trace("Client for " + languageId + " already exists! Skipping");
                }
            }
        }
    }

    wellnessCheck() {
       this.logger.trace("Running wellness check on " + this.activeClients.length + " active of " + this.clients.length + " total clients");
        let reconnectFlag = false;
        for (let client of this.activeClients) {
            if (!client.isProcessActive()) {
               this.logger.warn(client.getLanguagesString() + ": has no active process!");
                reconnectFlag = true;
                if (this.reconnects < 3) {
                   this.logger.info(client.getLanguagesString() + ": trying to reconnect");
                    client.stop();
                   this.logger.debug("Reconnecting");
                    client.connect();
                } else {
                   this.logger.warn(client.getLanguagesString() + ": max number of reconnects reached!");
                }
            }
        }
    
        if (reconnectFlag) {
            this.reconnects++;
        } else {
            this.reconnects = 0;
        }
    }
    
    deactivateAll(){
        for (let client of this.clients) {
            client.stop();
        }
    }

    activateWellnessCheck(interval: number){
	    this.timeout = setInterval(this.wellnessCheck, interval);
    }

}