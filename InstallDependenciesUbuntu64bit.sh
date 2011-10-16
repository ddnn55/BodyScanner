wget -O /tmp/of_preRelease_v007_linux64.tar.gz "http://www.openframeworks.cc/versions/preRelease_v0.07/of_preRelease_v007_linux64.tar.gz"
cd Libraries; tar xzvf /tmp/of_preRelease_v007_linux64.tar.gz; cd -
cd of_preRelease_v007_linux64/
cd Libraries/of_preRelease_v007_linux64/scripts/linux/ubuntu
sudo ./install_dependencies.sh
