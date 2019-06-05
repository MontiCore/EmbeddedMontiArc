#!/usr/bin/env bash
BASE_PATH="https://git.rwth-aachen.de/"

GITLAB_PRIVATE_TOKEN="zXKxi79T9iymW-FEHQgA"
GROUP_NAME=EmbeddedMontiArc

if [ -z "$GITLAB_PRIVATE_TOKEN" ]; then
echo "Please set the environment variable for GITLAB_PRIVATE_TOKEN"
echo "See ${BASE_PATH}profile/account"
exit 1
fi

echo "Cloning or pulling updates on all GitLab projects."

FIELD_NAME="X-Total-Pages"

TOTAL_PAGES=$(curl -s --head --header "PRIVATE-TOKEN: zXKxi79T9iymW-FEHQgA" https://git.rwth-aachen.de/api/v4/projects?membership=true | grep "$FIELD_NAME:" | grep -o [[:digit:]])

FIELD_NAME="http_url_to_repo"
REPO_GIT_URLS=""
for I in 1 .. $TOTAL_PAGES
do
    REPO_GIT_URLS=$REPO_GIT_URLS" "$(curl -s "${BASE_PATH}api/v4/projects?private_token=$GITLAB_PRIVATE_TOKEN&per_page=999&page=$I" | grep -o "\"$FIELD_NAME\":[^ ,]\+" | awk -F'"' '{print $4}' | grep $GROUP_NAME)
done

# echo ""
# echo $REPO_GIT_URLS
# echo ""

for REPO_GIT_URL in $REPO_GIT_URLS; do
    THEPATH=$(echo "$REPO_GIT_URL" | awk -F'/' '{print $NF}' | awk -F'.' '{print $1}')

    if [ ! -d "../EmbeddedMontiArc/$THEPATH" ]; then
        echo "Cloning $THEPATH ( $REPO_GIT_URL )"
        (cd "../EmbeddedMontiArc" && git clone "$REPO_GIT_URL" --quiet) &
    else
        echo "Pulling $THEPATH"
        (cd "../EmbeddedMontiArc/$THEPATH" && git pull --quiet) &
    fi
done
wait