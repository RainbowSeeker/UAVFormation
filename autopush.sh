#!/bin/bash

check_remote_update(){
    git fetch origin
    git diff --quiet HEAD origin/main
    if [ $? -eq 0 ]; then
        echo "No remote updates available."
    else
        echo "Remote Updates are available. Ready to pull..."
        git pull origin
    fi
}

check_local_update(){
    git diff --quiet HEAD .
    if [ $? -eq 0 ]; then
        echo "No local changes. Exiting..."
        read -s -n 1 -p "Press any key to exit..."
        exit 0
    else
        echo "Detect local change. Ready to push..."
        git diff --stat
    fi
}

check_remote_update
check_local_update

read -p "Please enter your commit: " commit
if [ -z "$commit" ]; then
    commit=`date +"%Y-%m-%d %H:%M:%S autopush"`
fi
git add . && echo "git commit -m \"$commit\"" | bash  && git push origin -f &&
read -s -n 1 -p "Press any key to exit..."
exit 0