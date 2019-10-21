#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
#!/bin/sh
BASE_PATH="https://git.rwth-aachen.de/"

GITLAB_PRIVATE_TOKEN=$DEPLOY_KEY
GROUP_NAME=$1

if [ ! -d $GROUP_NAME ]
then
  mkdir $GROUP_NAME
fi

if [ -z "$GITLAB_PRIVATE_TOKEN" ]; then
echo "Please set the environment variable for GITLAB_PRIVATE_TOKEN"
echo "See ${BASE_PATH}profile/account"
exit 1
fi

echo "Cloning or pulling updates on all GitLab projects."

FIELD_NAME="X-Total-Pages"

TOTAL_PAGES=$(curl -s --head --header "PRIVATE-TOKEN: ${GITLAB_PRIVATE_TOKEN} " "${BASE_PATH}api/v4/projects" | grep "$FIELD_NAME:" | grep -o [[:digit:]])

FIELD_NAME="path_with_namespace"
REPO_GIT_URLS=""
for I in 1 .. $TOTAL_PAGES
do
    REPO_GIT_URLS=$REPO_GIT_URLS" "$(curl -s "${BASE_PATH}api/v4/projects?membership=true&private_token=$GITLAB_PRIVATE_TOKEN&per_page=999&page=$I" | grep -o "\"$FIELD_NAME\":[^ ,]\+" | awk -F'"' '{print $4}' | grep $GROUP_NAME)
done

for REPO_GIT_URL in $REPO_GIT_URLS; do
    REPO_GIT_URL="https://oauth2:$GITLAB_PRIVATE_TOKEN@git.rwth-aachen.de/$REPO_GIT_URL.git"
    THEPATH=$(echo "$REPO_GIT_URL" | awk -F'/' '{print $NF}' | awk -F'.' '{print $1}')

    if [ ! -d "$GROUP_NAME/$THEPATH" ]; then
        echo "Cloning $THEPATH ( $REPO_GIT_URL )"
        (cd "$GROUP_NAME" && GIT_TERMINAL_PROMPT=0 git clone "$REPO_GIT_URL" --quiet) &
    else
        echo "Pulling $THEPATH"
        (cd "$GROUP_NAME/$THEPATH" && GIT_TERMINAL_PROMPT=0 git pull --quiet) &
    fi
done
echo "Successfull cloned projects:\r"
(cd "$GROUP_NAME" && ls)
wait
