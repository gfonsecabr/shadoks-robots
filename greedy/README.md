# CG:SHOP 2021

A software for computing maximum convex partitions of a set of points. These are some notes about the program.


## Compiling C++ code
The compiler for C++ is usually installed by default in Linux. Otherwise, do `sudo apt install build-essential`.

You may need to install cmake with `sudo apt install cmake` and maybe also the graphic version, `sudo apt install cmake-qt-gui`.

I recommend the IDE QtCreator. Install it with `sudo apt install qtcreator`. I lost a lot of time with a stupid error in the last version with the Clang model. You can disable it, read [this](https://stackoverflow.com/questions/52162027/qt-creator-warning-the-code-model-could-not-pass-an-included-file).


## Installing CPLEX
CPLEX is a library for solving linear programming problems. Follow these steps:
1. Download it from [IBM Academic Initiative](https://www.ibm.com/products/ilog-cplex-optimization-studio). Use Google CHromme and choose HTTp as download method. The current version is 12.10
2. Make the file executable with `chmod +x cplex_studio1210.linux-x86-64.bin`
3. Run the file to install it with `sudo ./cplex_studio1210.linux-x86-64.bin`
4. Use all the default parameters


## Connecting to other machines using SSH
- Connect to a different machine with `ssh gonzalez-l.a@147.94.182.166`
- I can add my SSH key with `ssh-copy-id -i id_ed25519.pub gonzalez-l.a@147.94.182.166` [source](https://linuxize.com/post/how-to-setup-passwordless-ssh-login/)
- I can send a file with `scp -Cp solver/aldo/build/cgshop2021 gonzalez-l.a@147.94.182.166:cg-shop-2021/`
- To execute a program in a machine, you will need [screen](http://www.gnu.org/software/screen/). You can do `ssh localhost -f 'screen -d -m sleep 60'` [source](https://superuser.com/questions/8673/how-can-i-use-ssh-to-run-a-command-on-a-remote-unix-machine-and-exit-before-the). I still do not know exactly how to do this.
- I can get the files with `scp -Cp gonzalez-l.a@147.94.182.166:cg-shop-2021/*sol.json .`

## Using screen
I assume that we only need one session. Then,
1. Launch session with `screen`
2. Once the job is launched, you can get out with `Ctrl + a` and then `d` for detach
3. Come back to the session with `screen -r`
Source: https://linuxize.com/post/how-to-use-linux-screen/
Also, 
- I can launch a named session wih `screen -S session_name`. Come back to that session with `screen -r session_name`. 
- I think that I can run a script inside a screen with `screen -d -m -S session_name jobs.sh`. An exemple of such a script is in the tools. I cannot execute directly a program.

---



## Read JSON files
Use RapidJSON (https://rapidjson.org/). See how to read a file [here](https://rapidjson.org/md_doc_stream.html).


## Warning
* The shuffling algorithm is not reproductible! I can maybe use random_suffle... http://www.cplusplus.com/reference/algorithm/random_shuffle/?kw=random_shuffle. Here (https://stackoverflow.com/questions/38067584/does-stdshuffle-guarantees-same-order-with-same-seed-on-different-vectors) says that even with the same seed, the shuffle can be different.


## Extracting the best results
For the moment, I do it by hand
1. Use the script extract-data.sh to extract the statistics from the JSON files
2. Copy the output file ("data") and paste it into the spreadhsheet ("results.odt")
3. Copy the rightmost 4 columns and paste them in Sublime Text 3
4. Remove the empty lines by replacing "\t\t\n" by "" (using regexes)


## Cluster du LIS
* Mes identifiants sont les mêmes que pour l'intranet du LIS. Je peux voir l'état des machines ici : (https://cluster.lis-lab.fr/drawgantt/). Pour me connecter avec la console (je n'ai pas réussi avec FileZilla), faire : `ssh aldo.gonzalez-lorenzo@saphir2.lis-lab.fr` puis `ssh frontend`. C'est là que je mets mes fichiers.  
* Pour envoyer l'executable, faire :
```
cd Dropbox/DropboxAldo/Software/convex-partition/mcp-graph/build/
scp mcp-graph aldo.gonzalez-lorenzo@saphir2.lis-lab.fr:
ssh aldo.gonzalez-lorenzo@saphir2.lis-lab.fr
scp mcp-graph frontend:
```
* I can compress a folder with `tar -zcvf archive.tar.gz directory/ ` and decompress it with `tar -zxvf archive.tar.gz` ([source](https://unix.stackexchange.com/questions/93139/can-i-zip-an-entire-folder-using-gzip)).
* To send a (passive) job: `oarsub -p "(gpu IS NULL)" -l "walltime=10:0:0" "./mcp-graph data/uniform/uniform-0001000-1.instance.json data/uniform/solution-ilpm0/uniform-0001000-1.solution-ilpm0.json 3 --no-svg"`
* To send many jobs: `oarsub -p "(gpu IS NULL)" -l "walltime=10:0:0" --array-param-file param_combinations.txt "./mcp-graph"`
* To make the parameters file, use the script `tools/write-parameters.sh`. Then, copy this, and do `vim param_combinations.txt` ; `i` ; Ctrl+Shift+V
* To check my jobs: `oarstat`


## Using GitLab
Go to https://gitlab.lis-lab.fr/ and log in (LDAP) with my LIS account. I had to add a SSH key before anything. Go to the parent folder and do `git clone git@gitlab.lis-lab.fr:aldo.gonzalez-lorenzo/mcp-graph.git`. Go inside and do `git pull origin master`. Then, `git add .`, `git commit -m "First commit"` and `git push origin master`.

## Improving CPLEX
* Read [this](https://www.ibm.com/support/pages/cplex-performance-tuning-mixed-integer-programs): It is not long and it gives good hints, but I have to find how to code them.
* [This beautiful paper](http://inside.mines.edu/~anewman/MIP_practice120212.pdf) explains the branch-and-bound algorithm for solving ILP. We have to read the log to see if the algorithms for solving relaxed LP are efficient. Also, giving solutions is always good, even if they are bad.
* Read [this](https://www.ibm.com/support/knowledgecenter/SSSA5P_12.7.0/ilog.odms.cplex.help/CPLEX/UsrMan/topics/discr_optim/mip/para/52_node_log.html) for understanding the log. Same for [this](https://www.ibm.com/support/knowledgecenter/SSSA5P_12.5.1/ilog.odms.ide.help/OPL_Studio/usroplide/topics/opl_ide_stats_MP_exam_log.html).
* [This](https://www.ibm.com/support/pages/sites/default/files/support/swg/dmgtech.nsf/0/85256dd00053125a8525762700223102/%24FILE/RethinkingMixedIntegeModelFormulations.pdf) is about using different models for the same problem.