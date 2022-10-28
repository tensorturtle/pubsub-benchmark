echo "Cloning 'tensorturtle/cereal' (a fork of `commaai/cereal`) from Github"
git clone https://github.com/tensorturtle/cereal.git
echo "Building Docker container"
cd cereal
sudo docker build -t tensorturtle/cereal .
echo "Deleting downloaded repository"
cd ..
rm -rf cereal
