# Race line Generation instruction

## enviournmnet setup
1. clone the following repo in your wato workspace or code directory : git@github.com:WATonomous/Raceline-Optimization.git
2. download conda on your system, this is the best way to wrangle all the dependencies 
3. make a new conda venv using the following command : `conda create -n wato_env python=3.8` (very import that you use 3.8 as otherwise the dependencies break)
4. run : `conda activate wato_env`
5. cd into the raceline optimization folder and run : `pip install -r requirements.txt`
6. use the `map_converter` to turn the pgm + yaml into a center line 
7. verify that is correct using sanity check
8. make the optimal line using the `main_globaltraj_f110.py` (you will need to adjust the map variable to make it yours in this one)
