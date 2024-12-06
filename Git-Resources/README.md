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
name@device-name:~$ ls
Desktop  Documents  Downloads  Music  Pictures  Public  snap  Templates  Videos
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
name@device-name:~/Code/my-frist-repo$ git status
On branch main

No commits yet

nothing to commit (create/copy files and use "git add" to track)
```

## Managing Your Git Repository

Now that we have a git repository, now lets add some stuff to it!

### Adding Files

We can create files with the terminal command `touch`. Lets create a new file:

```
touch my-first-file.txt
```

Now if we do a `git status` we will see the following:

```
name@device-name:~/Code/my-frist-repo$ git status
On branch main

No commits yet

Untracked files:
  (use "git add <file>..." to include in what will be committed)
	my-first-file.txt

nothing added to commit but untracked files present (use "git add" to track)
```

We can now add this file with the following git command:

```
git add my-first-file.txt
```

After adding the file, the `git status` should output something simular to this:

```
name@device-name:~/Code/my-frist-repo$ git status
On branch main

No commits yet

Changes to be committed:
  (use "git rm --cached <file>..." to unstage)
	new file:   my-first-file.txt

```

### Removing Files

Egad! I did not want to stage that file!  

To unstage a file, you use the following command:

```
git rm --cached my-first-file.txt
```

### Commiting Files

What is a commit? 

A Git commit is a snapshot of changes made to the files in a repository at a specific point in time. It represents a version of the project, allowing you to track, save, and manage the project's history.

Let us re-add the file with `git add`. After making sure that the file is is staged with `git status`, we are ready to commit our changes. To do this, we use the following command:

```
git commit -m "my first commit"
```

When we are making a commit we have to leave a message with the `-m` flag as well as a message in `"`s.  

The output of the commit should be simular to the following:

```
name@device-name:~/Code/my-frist-repo$ git commit -m "my first commit" 
[main (root-commit) 61723e7] my first commit
 1 file changed, 0 insertions(+), 0 deletions(-)
 create mode 100644 my-first-file.txt
```

## Using Remote Repositories

### What is a Remote Repository?

A remote repository is a project version that is stored on a centralized server or cloud-based platform, and shared by multiple team members.

Have you heard of [GitHub](https://github.com)? 

Well GitHub is the remote repository service that CSM uses. Another example of remote repository service would be GitLab, which is what is used by the Computer Science department.

### Setting Git Up For Remote Repositories

If you have not already set up your local git so that you can access the remote repositories that are stored on GitHub, you can do so with the following commands:

#### Set Your Name

```
git config --global user.name "John Doe"
```

You are going to want to set your name so that your peers will be able to know who is makeing changes to the code inline.

#### Set Your Email

```
git config --global user.email "johndoe@email.com"
```

You are going to want ot set your email to one that is linked to your GitHub account, I would also recomend adding your school email to your GitHub account so that it will be easy to add your schoolwork at a later date. I would also recomend you to use the autogenerated email that can be found in your GitHub account email settings, this is an example of what it could look like: `123456789+johndoe@users.noreply.github.com`

### Setting Up A Remote Repository

We're going to be setting up your first remote repository.

#### Step 1: Create Remote Repository

Go to github.com and after signing in there should be an option on the left side that says "New". Click that. This should take you to a new page titled `Create a new repository`. From here we can create the repository. Lets title it something simple like `remote-repo`. In this menu it allows you to decide whether to make the repository public or private, lets make our public for simplicity, we can always private or delete this repository later.

#### Step 2: Cloning Your Remote Repository

Now that we have a remote repo we can clone it to our local machine. Lets `cd` into our `Code` directory so that we can clone our repository. We can do this with the following `git clone` command, we use the following syntax (you will have to replace johndoe with your username):

```
git clone git@github.com:johndoe/remote-repo.git
```

This should have an output which is simular to this:

```
name@device-name:~/Code$ git clone git@github.com:johndoe/remote-repo.git
Cloning into 'remote-repo'...
warning: You appear to have cloned an empty repository.
```

Congrats! We now have a cloned (empty) repository.
