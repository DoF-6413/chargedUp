# chargedUp
Code base for 2023 season DOF

## How to Access Code
First Open Git Bash

- Type in eval "$(ssh-agent -s)" && ssh-add ~/.ssh/[name]_key
- CD Into your Repo
	- Open File Explorer
	- Open Folder you are trying to Access
	- Right Click Top Bar with Folders you are In
	- Copy Address
	- type: cd [paste address here]
- Open VS Code and Enter Repo


## Branch Organization

main Branch: 

- Use for Competitions
- Only Receives Pushes from Dev

dev Branch:

-  Use for Full Systems Tests
- Only Push to Main after Tested and Clean
	- No hardcoded values
	-  Explanatory Comments
	- Code is Fully Functioning
-  Make Feature Branches off of *this* Branch

feat-[name]: 

-  Branch Name is all Lowercase
- Make Feature Branches off of the Dev Branch
-  Use for and Code changes
	- Changes can be as small or big as necessary
	- Only Contains what the Branch is named
		- EX. No climber code in a feat-shooter branch
-  Push to Dev when code is Tested and Clean
	- No hardcoded values
	-  Explanatory Comments
	- Code is Fully Functioning
