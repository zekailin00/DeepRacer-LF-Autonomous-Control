# DeepRacer LF Automonous Control Dev Repo

## Development Procedure 

Both ROS program and LF program are compiled on a Linux machine. After build success, the LF binaries are commited to the repo and pulled onto the car. 

## Develope LF program

`ros-workspace` ROS pacakges can be added into this directory. 

`lf-workspace` LF files can be added into this directory. Usually, one LF file matches one ROS pacakge inside `ros-workspace`

`. lfcbuild.sh` compiles LF program on the Linux machine automatically, it has all ROS environment dependencies included and rebuilds ROS packages every time it is run. 

`. clear.sh` It removes all LF and ROS output files generated from running `. lfcbuild.sh`

## Deploy LF program onto the car 
Since `lf-workspace/bin` folder is not added in `.gitignore`, LF program binaries can be pushed and pulled to a DeepRacer car

Modify `./Main-car` file to make the environment path match what's inside the car.

 `git add -A && git commit -m"upload" && git push` on Linux

 `git pull` on DeepRacer

 `. lfcarrun.sh` on DeepRacer
