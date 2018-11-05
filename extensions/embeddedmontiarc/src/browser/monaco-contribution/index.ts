/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { EMBEDDEDMONTIARC_LANGUAGE_ID, EMBEDDEDMONTIARC_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./embeddedmontiarc-monaco-language";

monaco.languages.register({
    id: EMBEDDEDMONTIARC_LANGUAGE_ID,
    extensions: [".ema"],
    aliases: [EMBEDDEDMONTIARC_LANGUAGE_NAME],
    mimetypes: ["text/x-embeddedmontiarc"]
});

monaco.languages.onLanguage(EMBEDDEDMONTIARC_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(EMBEDDEDMONTIARC_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(EMBEDDEDMONTIARC_LANGUAGE_ID, monarchLanguage);
});
