#!/bin/bash
set -e # Exit with nonzero exit code if anything fails

SOURCE_BRANCH="master"
TARGET_BRANCH="master"


# Save some useful information
REPO=`git config remote.origin.url`
SSH_REPO=${REPO/https:\/\/github.com\//git@github.com:}
SHA=`git rev-parse --verify HEAD`

git config user.name "Travis CI"
git config user.email "$COMMIT_AUTHOR_EMAIL"

git add *.zip
git commit -m "Deploy new Zips: ${SHA}"

# Now that we're all set up, we can push.
git push $SSH_REPO $TARGET_BRANCH