# download source code of neuron/interview
curl -o nrn.tar.gz https://neuron.yale.edu/ftp/neuron/versions/v7.6/nrn-7.6.tar.gz
curl -o iv.tar.gz https://neuron.yale.edu/ftp/neuron/versions/v7.6/iv-19.tar.gz

# extract downlaoded tarballs
tar -xvzf nrn.tar.gz
tar -xvzf iv.tar.gz

# remove tarballs
rm *.tar.gz

# move extracted source code to 
mkdir ~/neuron
mv nrn-7.6 ~/neuron/nrn
mv iv-19 ~/neuron/iv