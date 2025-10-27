#!/usr/bin/env python3

"""
Git Wrapper for QIRP Device

This script provides basic git functionality using GitPython since the git CLI
is not available via opkg on QIRP devices.
"""

import sys
import os
import argparse
from pathlib import Path

try:
    import git
except ImportError:
    print("Error: GitPython not installed. Run: pip3 install GitPython --break-system-packages")
    sys.exit(1)

def git_init(directory="."):
    """Initialize a git repository"""
    try:
        repo = git.Repo.init(directory)
        print(f"Initialized empty Git repository in {os.path.abspath(directory)}")
        return True
    except Exception as e:
        print(f"Error initializing repository: {e}")
        return False

def git_status(directory="."):
    """Show git status"""
    try:
        repo = git.Repo(directory)
        print("On branch", repo.active_branch.name)
        
        # Show untracked files
        untracked = repo.untracked_files
        if untracked:
            print("\nUntracked files:")
            for file in untracked:
                print(f"  {file}")
        
        # Show modified files
        modified = [item.a_path for item in repo.index.diff(None)]
        if modified:
            print("\nModified files:")
            for file in modified:
                print(f"  {file}")
        
        # Show staged files
        staged = [item.a_path for item in repo.index.diff("HEAD")]
        if staged:
            print("\nStaged files:")
            for file in staged:
                print(f"  {file}")
        
        return True
    except Exception as e:
        print(f"Error getting status: {e}")
        return False

def git_add(files, directory="."):
    """Add files to git index"""
    try:
        repo = git.Repo(directory)
        if files == ["."] or files == ["all"]:
            repo.git.add(".")
            print("Added all files")
        else:
            for file in files:
                repo.git.add(file)
                print(f"Added {file}")
        return True
    except Exception as e:
        print(f"Error adding files: {e}")
        return False

def git_commit(message, directory="."):
    """Commit changes"""
    try:
        repo = git.Repo(directory)
        repo.index.commit(message)
        print(f"Committed: {message}")
        return True
    except Exception as e:
        print(f"Error committing: {e}")
        return False

def git_log(directory=".", count=10):
    """Show git log"""
    try:
        repo = git.Repo(directory)
        commits = list(repo.iter_commits(max_count=count))
        
        for commit in commits:
            print(f"commit {commit.hexsha[:8]}")
            print(f"Author: {commit.author.name} <{commit.author.email}>")
            print(f"Date: {commit.committed_datetime}")
            print(f"    {commit.message.strip()}")
            print()
        
        return True
    except Exception as e:
        print(f"Error getting log: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Git wrapper for QIRP device")
    parser.add_argument("command", choices=["init", "status", "add", "commit", "log"], 
                       help="Git command to execute")
    parser.add_argument("args", nargs="*", help="Command arguments")
    parser.add_argument("-C", "--directory", default=".", help="Git directory")
    
    args = parser.parse_args()
    
    if args.command == "init":
        git_init(args.directory)
    elif args.command == "status":
        git_status(args.directory)
    elif args.command == "add":
        if not args.args:
            print("Error: No files specified for add command")
            sys.exit(1)
        git_add(args.args, args.directory)
    elif args.command == "commit":
        if not args.args:
            print("Error: No commit message specified")
            sys.exit(1)
        message = " ".join(args.args)
        git_commit(message, args.directory)
    elif args.command == "log":
        count = int(args.args[0]) if args.args else 10
        git_log(args.directory, count)

if __name__ == "__main__":
    main()
