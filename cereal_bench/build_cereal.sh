echo "Cloning 'commaai/cereal' from Github"
git clone https://github.com/commaai/cereal.git
echo "Building Docker container"
sudo docker build -t tensorturtle/cereal .
echo "Deleting downloaded repository"
rm -rf cereal
