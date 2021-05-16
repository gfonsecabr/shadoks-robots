# Greedy solver

Software for computing step-by-step solutions with a greedy approach.


## Installing CPLEX
CPLEX is a library for solving linear programming problems. Follow these steps to install it:
1. Download it from [IBM Academic Initiative](https://www.ibm.com/products/ilog-cplex-optimization-studio). Use Google Chromme and choose HTTP as download method. The current version is 12.10
2. Make the file executable with `chmod +x cplex_studio1210.linux-x86-64.bin`
3. Run the file to install it with `sudo ./cplex_studio1210.linux-x86-64.bin`
4. Use all the default parameters

## Installation
Go to this folder and do
```
mkdir build
cd build
cmake-gui ..
```
Once CPLEX is installed, if the path is not found you can set the variable `CPLEX_ROOT_DIR` to `/opt/ibm/ILOG/CPLEX_Studio1210`. Click on *Configure* a few times, then on *Generate*. Afterwards, compile the code with `make`.