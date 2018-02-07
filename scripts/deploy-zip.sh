#!/bin/bash
set -e # Exit with nonzero exit code if anything fails

emaZip=$1
msZip=$2

SOURCE_BRANCH="master"
TARGET_BRANCH="gh-pages"

echo "EMAZIP: $emaZip"

# Save some useful information
REPO=`git config remote.origin.url`
SSH_REPO=${REPO/https:\/\/github.com\//git@github.com:}
SHA=`git rev-parse --verify HEAD`

if [ -d "out" ]
then
  rm -rf out
fi
git clone $REPO out
cd out
git checkout $TARGET_BRANCH || git checkout --orphan $TARGET_BRANCH
git config user.name "Travis CI"
git config user.email "$COMMIT_AUTHOR_EMAIL"
git rm *.zip

mv ../$emaZip $emaZip
mv ../$msZip $msZip

git add *.zip
git commit -m "Deploy new Zips: ${SHA}"

# Now that we're all set up, we can push.
git push $SSH_REPO $TARGET_BRANCH

cd ..