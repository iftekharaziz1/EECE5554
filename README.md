
EECE 5554 Sec. 01 Lab 0 v1.0 Fall 2024
1
LAB 0: Getting familiar with Linux, Git, and ROS
Due Friday, Sept. 13 by 11:59pm on GitHub
After completing this lab, a well-prepared student should be able to:
• Use basic functions of Ubuntu from the command line (make directories and files)
• Push files to a Git repository
• Write ROS nodes in Python
ROS Basics you will learn about in this assignment
 The ROS core systems: Packaging, Buildsystems, Messaging, CLI & GUI tools, creating
packages, nodes, topics, services, and parameters.
 Writing a Simple Publisher and Subscriber (Python)
 Examining the Simple Publisher and Subscriber
 Writing a Simple Service and Client (Python)
 Examining the Simple Service and Client
1. Install Ubuntu 20.04 LTS on your laptop. You can dual-boot or use virtualization software (Oracle
Virtualbox or UTM). Please make sure that your USB port and network connection are working. VM
instructions are at the end of this document. If you are making changes to your machine, it would be
a good idea to back up beforehand!
2. If you need a refresher or have not used Git before, do “good for beginners” starred tutorials on
GitHub. https://docs.github.com/en/get-started/quickstart/hello-world
3. Once you are comfortable with basic Git use through the command line/terminal, open an account on
GitHub using your Northeastern email ([name]@northeastern.edu) email (please!). Make sure you are
signing up for a free account.
Make your username the same as your Northeastern username if possible, and if not, make it a few
more letters. Example: dorsey-k (if available) or dorsey-kri
Post your username, if it differs from your Northeastern username, to the Piazza thread.
NOTICE:
It is essential that you:
1. Use your Northeastern email for your GitHub account
2. Match spelling and capitalization of subfolders with the instructions.
We cannot grade your assignments without these, and you will not receive credit.
4. On GitHub, create a repository called EECE5554 (Repository must be kept PRIVATE)
5. On GitHub, make a subdirectory under EECE5554 called LAB0
6. Install Git on your machine: https://GitHub.com/git-guides/install-git
EECE 5554 Sec. 01 Lab 0 v1.0 Fall 2024
2
7. Under the LAB0 subdirectory, use the command line interface to check in a text file named
[yourNortheasternunsername.txt] that states that you have:
1. A complete Ubuntu 20.04 LTS install or your plans to resolve any issues ASAP
2. You are ready to use Git and GitHub
If you can’t yet install Ubuntu, you can use the web interface to check this file in.
8. Invite course staff to be developers on your GitHub using email ECERSN@Northeastern.edu
9. Read about ROS:
 ROS Introduction
 ROS Get Started Guide Introduction, Concepts, Higher-Level Concepts, Client Libraries,
Technical Overview
 ROS developer's guide
 ROS Python Style Guide
 Useful cheat-sheets:
 ROS cheat-sheet for Kinetic and Catkin
 Documentation on command line tools: https://wiki.ros.org/ROS/CommandLineTools
 rosnode
 rostopic
 rosparam
 rosmsg
10. Install ROS Noetic http://wiki.ros.org/noetic/Installation/Ubuntu in Ubuntu
You can choose desktop full or desktop
11. Complete these Tutorials (* indicates a component of the assignment turn in)
NOTE: we will exclusively use Ubuntu 20.04 LTS and the ROS Noetic Release
 Installing and Configuring Your ROS Environment.
 Navigating the ROS Filesystem
 Creating a ROS Package
 Building a ROS Package
 Understanding ROS Nodes
 Understanding ROS Topics*
 Understanding ROS Services and Parameters
 Using rqt_console and roslaunch
 Using rosed to edit files in ROS
 Creating a ROS msg and a ROS srv
 Writing a Simple Publisher and Subscriber (Python)*
 Examining the Simple Publisher and Subscriber
EECE 5554 Sec. 01 Lab 0 v1.0 Fall 2024
3
Useful GitHub Links
Cloning Remote Repositories
https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories#cloning-with-
https-urls
Personal Access Tokens
https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-
personal-access-tokens
Authentication
https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-
personal-access-tokens#creating-a-personal-access-token-classic
https://docs.github.com/en/get-started/signing-up-for-github/verifying-your-email-address
Deleting a Repository
https://docs.github.com/en/repositories/creating-and-managing-repositories/deleting-a-repository
What to submit:
 Understanding ROS Topics: submit a screenshot of your turtlesim window with the turtle having
drawn a small doodle (your first initial, a star, etc.)
 Writing a Simple Publisher and Subscriber: Starting with the talker/listener example, write and
test a publisher node that publishes a message containing a string (anything but hello world) to
the topic chatter and a subscriber node that subscribes to the topic chatter with the callback “I
heard” + the string message, where the string message is slightly modified in some way (two
letters flipped, append a character to the string, etc.).
EECE 5554 Sec. 01 Lab 0 v1.0 Fall 2024
4
Submission Structure:
Your GitHub repository should be accessible using the https://github.com/<username>/EECE5554
Your GitHub repository should look like this structure when cloned locally:
EECE5554/LAB0/src/[files]
EECE5554
LAB0
[Northeasternusername.txt]
src
[files from ROS tutorial]
Assessment:
Points
Made GitHub account with your NU email & posted to Piazza thread if your GitHub
username is not the same as your Northeastern username
10
Invited ECERSN@Northeastern.edu to be a developer to your GitHub repo 10
Correct GitHub repository structure (EECE5554/LAB0/[Northeasternusername.txt]) 10
Your text file says you installed Ubuntu (or states any challenges you are having and
how you plan to fix them)
10
Your text file states that you installed Git 10
Screenshot of the turtle having drawn a fun doodle 10
Modified talker node 15
Modified listener node 15
Correct Lab 0 structure & upload to GitHub using the command line interface (CLI) 10
Total 100
You should attempt to use the command line interface (CLI) to push to your GitHub repo. Please ask for
help if you are unsure. If there are errors in your LAB0 structure on GitHub, you may fix and resubmit
for this assignment for full credit. However, for future assignments, you will need to use the command
line interface, so we recommend you get used to it now!
EECE 5554 Sec. 01 Lab 0 v1.0 Fall 2024
5
VirtualBox instructions for Ubuntu (Intel-based Macs & Windows):
1. Download the .iso for Ubuntu 20.04 LTS (long term service).
https://releases.ubuntu.com/focal/
2. Install Virtual Box or other virtualization software: https://www.virtualbox.org/
3. Use these instructions to set up a Ubuntu machine with Virtual Box: https://ubuntu.com/tutorials/how-
to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview
4. I recommend 2048+ MB RAM, 25+ GB of hard disk space, and it doesn’t matter if you choose fixed
or dynamic virtual drive
5. Be sure to check out the last page of instructions to understand how to resize the window!
UTM instructions for Ubuntu (Apple M1/M2 ARM processor Macs):
1. Download the .iso for Ubuntu 20.04 daily build: https://cdimage.ubuntu.com/focal/daily-live/pending/
2. Install UTM from the Apple store (price was $10 in Fall 2022)
3. Use these instructions to set up a Ubuntu machine with Virtual Box: https://ubuntu.com/tutorials/how-
to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview
4. I recommend 2048+ MB RAM, 25+ GB of hard disk space, and it doesn’t matter if you choose fixed
or dynamic virtual drive
5. After you complete installation, make sure you “eject” the *.iso from the virtual machine, or it will
ask if you want to install again.
