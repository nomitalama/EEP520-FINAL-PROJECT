# EEP520-FINAL-PROJECT
This program genarates a random maze everytime it is ran. Then Binky (the robot), will solve the maze by folowing the right walls. It may not seem like the most clever method, but it's a fool-proof way to get through.  

# Installation:
Simply pull from the repo (https://github.com/nomitalama/EEP520-FINAL-PROJECT) which will contain folder for "my_project" and it should have all the files needed. 

Run docker using the following command: *docker run -p80:80 -p8765:8765 -v /C/Users/admin/Desktop/final_assignment/520-Assignments/hw_8b:/source -it klavins/enviro:alpha bash*

Modify the above command to match the address where you've saved your files. 

Use the command *esm start* to stard a web server using enviro. Search http://localhost with your web browser and see the ENVIRO client in a new tab. 

To run the files use the commands below:

*make*

*enviro*

After the initial *make*, every time the user inputs *enviro*, a new maze will randomly form. 
