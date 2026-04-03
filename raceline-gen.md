# Race line Generation instruction

## enviournmnet setup
1. download conda on your system, this is the best way to wrangle all the dependencies 
2. make a new conda venv using the following command : `conda create -n wato_env python=3.8` (very import that you use 3.8 as otherwise the dependencies break)
3. run : `conda activate wato_env`
4. cd into the raceline optimization folder and run : `pip install -r requirements.txt`
5. use the `map_converter` to turn the pgm + yaml into a center line 
6. verify that is correct using sanity check
7. make the optimal line using the `main_globaltraj_f110.py` (you will need to adjust the map variable to make it yours in this one)
