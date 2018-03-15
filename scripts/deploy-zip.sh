#!/bin/bash
set -e # Exit with nonzero exit code if anything fails
cd out

git rm *.zip

for var in "$@"
do
    mv ../$var $var
done

git add *.zip
git commit -m "Deploy new Zips: ${SHA}"

git push $SSH_REPO $TARGET_BRANCH
cd ..
