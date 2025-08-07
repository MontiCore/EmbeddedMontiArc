/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { CD_LANGUAGE_ID, CD_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./cd-monaco-language";

monaco.languages.register({
    id: CD_LANGUAGE_ID,
    extensions: [".cd"],
    aliases: [CD_LANGUAGE_NAME],
    mimetypes: ["text/x-cd"]
});

monaco.languages.onLanguage(CD_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(CD_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(CD_LANGUAGE_ID, monarchLanguage);
});
