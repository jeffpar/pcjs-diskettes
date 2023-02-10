## PCjs System Diskette Repository

The [PCjs System Diskette Repository](https://github.com/jeffpar/pcjs-diskettes) is a collection of system diskette images
for use with the [PCjs Machines](https://www.pcjs.org) website.  They are preserved here for archival/demonstration purposes only.

{% comment %}

I used this suggestion from [stackoverflow](https://stackoverflow.com/questions/9683279/make-the-current-commit-the-only-initial-commit-in-a-git-repository) to clean up the repository after migrating a large number of files to another repository:

    git checkout --orphan newBranch
    git add -A  # Add all files and commit them
    git commit
    git branch -D master  # Deletes the master branch
    git branch -m master  # Rename the current branch to master
    git gc --aggressive --prune=all     # remove the old files
    git push -f origin master  # Force push master branch to github

This reduced the size of the repository on GitHub by more than half, so it was worthwhile.

I didn't really want to lose *all* the history -- just the history of the files that were no longer part of the repository.  But on the other hand, all the JSON-encoded disk images had grown slightly after upgrading them to version 2.10, so removing the history for all files was probably for the best.

{% endcomment %}
