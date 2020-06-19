# navigate to iv
cd ~/neuron/iv
./configure --prefix=`pwd`

# make iv
make
make install

# navigate to neuron
cd ~/neuron/nrn
python_loc=$(which python)
./configure --prefix=`pwd` --with-iv=$HOME/neuron/iv --with-paranrn --with-nrnpython=$python_loc

# make neuron
make
make install
