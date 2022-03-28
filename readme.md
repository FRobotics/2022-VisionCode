# Frobotics-2022-VisonCode
---------------------------

Frobotics 4150 vision code for 2022

This was copied from DariusAC 2020 vision code (which has the 2022 vision code)

### To install the compiler on windows do the following:

1) Download the zip file from:
	https://github.com/wpilibsuite/raspbian-toolchain/releases

2) Copy the unzipped files to:
	 C:\raspbian10

3)  Go to the start menu and search for “environment variables” and there should be 
an option to Edit the system environment variables (in Control Panel). Edit 
the PATH variable and add C:\raspbian10\bin to it (it’s a semicolon-delimited list).

### To compile

1) After performing the install -- open a windows command prompt.

2) Navigate to the "code" directory under the github repository's root directory.

3) Issue the command:
	make build

### To remove existing compiled versions (to force a new compile next time make build is run)

1) Open a windows command prompt.

2) Navigate to the "code" directory under the github repository's root directory.

3) Issue the command:
	make clean


### To update the executable on the raspberry PI

1) Open the Raspberry pi web page (wpilibpi.local)

2) Go to the application page.  

3) At the top press "writable"

4) In the top section, select "Uploaded C++ executable"

5) Browse to find and select the compiled file.  It is named "multiCameraServerExample".
It will be in the same directory as main.cpp

5) Press "Upload and save"

6) Go to the "Vision Status" page, and turn on the console to see any messages.

### Compatibility
This vision program was written using the 2021.3.1 libraries.  It works with that version 
of the raspberry pi operating system image.  It may or may not work with other versions.

