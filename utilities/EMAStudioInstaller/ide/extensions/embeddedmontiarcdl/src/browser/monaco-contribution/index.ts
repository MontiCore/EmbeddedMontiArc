/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { EMBEDDEDMONTIARCDL_LANGUAGE_ID, EMBEDDEDMONTIARCDL_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./embeddedmontiarcdl-monaco-language";

monaco.languages.register({
    id: EMBEDDEDMONTIARCDL_LANGUAGE_ID,
    extensions: [".emadl"],
    aliases: [EMBEDDEDMONTIARCDL_LANGUAGE_NAME],
    mimetypes: ["text/x-embeddedmontiarcdl"]
});

monaco.languages.onLanguage(EMBEDDEDMONTIARCDL_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(EMBEDDEDMONTIARCDL_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(EMBEDDEDMONTIARCDL_LANGUAGE_ID, monarchLanguage);
});
