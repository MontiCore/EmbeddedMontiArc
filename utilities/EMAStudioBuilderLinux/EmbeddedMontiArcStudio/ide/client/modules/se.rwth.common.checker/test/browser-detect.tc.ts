/* (c) https://github.com/MontiCore/monticore */
import {ClientFunction} from "testcafe";

declare var browser;

export class BrowserDetectTestController {
    /*
     * Methods
     */
    public async browser(): Promise<any> {
        return ClientFunction(() => browser()).apply(null, arguments);
    }
}
