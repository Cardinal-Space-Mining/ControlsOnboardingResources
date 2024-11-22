# Git Resources

Welcome to the **Git Resources** directory! This directory contains resources for beginner git users to practice actually using git! 

## Overview

This directory aims to get new git users familiar with using the version control system git. This directory will be covering the following:

- Initializing git repositories
- Managing your git repositories
	- adding files
	- commiting files
- Cloning remote git repositories
- Using a remote repository

## Getting Started With Git

If you have followed the [Evironment Setup](https://github.com/Cardinal-Space-Mining/CSMWiki/tree/main/EnviromentSetup) steps then you should have git installed, if you have yet to complete the environment setup you can use the following linux terminal command (requires Ubuntu):

```
sudo apt update
sudo apt install git-all
```

## Initializing a Git Repository

After getting git, you now are able to create a git repository. To do this open the terminal, with Ubuntu you can do this with the keyboard shortcut `Ctrl+Shift+T`. Now that you have opened the terminal, use the following command to view the contents of your home directory witht the terminal command `ls`. The terminal should output something simular to this:

```
THIS NEEDS TO BE UPDATED, DO NOT FORGET TO UPDATE THIS
```

we will create a `Code` directory with the following command:

```
mkdir Code
```

This will create an easy to find place for all of your code! We are now going to change to the newly create `Code` directory with the following terminal command:

```
cd Code
```

To return to your home directory, you can use the terminal command `cd` with no target.  

Now that we are in the `Code` directory, create a new directory:

```
mkdir my-first-repo
```

Now that you have a new directory, `cd` into the directory with the following command:

```
cd my-first-repo
```

Now that you are in the new directroy, we can initialize the git repository with the command:

```
git init
```

Congrats, now you have a initialized a git repository! You can check that everything is in order by typing the command:

```
git status
```

It should output somthing simular to the following:
```
name@device:my-first-repo$ git status
On branch main

No commits yet

nothing to commit (create/copy files and use "git add" to track)
```

## Managing Your Git Repository