/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { EMBEDDEDMONTIVIEW_LANGUAGE_ID, EMBEDDEDMONTIVIEW_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./embeddedmontiview-monaco-language";

monaco.languages.register({
    id: EMBEDDEDMONTIVIEW_LANGUAGE_ID,
    extensions: [".emv"],
    aliases: [EMBEDDEDMONTIVIEW_LANGUAGE_NAME],
    mimetypes: ["text/x-embeddedmontiview"]
});

monaco.languages.onLanguage(EMBEDDEDMONTIVIEW_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(EMBEDDEDMONTIVIEW_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(EMBEDDEDMONTIVIEW_LANGUAGE_ID, monarchLanguage);
});
