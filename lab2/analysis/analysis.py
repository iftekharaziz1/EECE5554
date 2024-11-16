#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_and_calculate_deviation(file_path, dataset_title, deviation_dict):
    df = pd.read_csv(file_path)
    
    #2D scatterplot- Easting vs. Northing
    x_coords = df['UTM_easting']
    y_coords = df['UTM_northing']
    plt.figure(figsize=(8, 6))
    plt.scatter(x_coords, y_coords, label="GPS Data", c='blue', s=10)
    plt.xlabel('Easting (m)')
    plt.ylabel('Northing (m)')
    plt.title(f'2D Scatter Plot - {dataset_title}')
    plt.legend()
    plt.grid(True)
    plt.show()
# Calculate mean positions
    mean_x = np.mean(x_coords)
    mean_y = np.mean(y_coords)
    
    deviation_dict[dataset_title] = np.sqrt((x_coords - mean_x)**2 + (y_coords - mean_y)**2)
    avg_deviation = np.mean(deviation_dict[dataset_title])
    
    print(f"Mean position for {dataset_title}: Easting={mean_x:.2f}, Northing={mean_y:.2f}")
    print(f"Mean deviation from the mean position for {dataset_title}: {avg_deviation:.2f} meters")

    return mean_x, mean_y, avg_deviation

def plot_deviation_bar_chart(deviation_dict):
    plot_titles = list(deviation_dict.keys())
    avg_deviations = [np.mean(deviation_dict[title]) for title in plot_titles]
    
    plt.figure(figsize=(8, 6))
    plt.bar(plot_titles, avg_deviations, color=['blue', 'orange', 'green', 'red'])
    plt.xlabel('Dataset')
    plt.ylabel('Mean Deviation (meters)')
    plt.title('Mean Deviation for Each Dataset')
    plt.xticks(rotation=15)
    plt.grid(axis='y')
    plt.show()

if __name__ == "__main__":
    csv_file = [
        '/home/iftekhar/EECE5554/lab2/data/open_stationary/open_stationary.csv',
        '/home/iftekhar/EECE5554/lab2/data/occluded_stationary/occluded_stationary.csv',
        '/home/iftekhar/EECE5554/lab2/data/open_move/open_move.csv',
        '/home/iftekhar/EECE5554/lab2/data/occluded_move/occluded_move.csv' 
    ]
    
    # plots Title
    plot_titles = ['Open Stationary', 'Occluded Stationary', 'Open Moving', 'Occluded Moving']
    
    deviation_dict = {}
    
    for file_path, dataset_title in zip(csv_file, plot_titles):
        plot_and_calculate_deviation(file_path, dataset_title, deviation_dict)
    
    plot_deviation_bar_chart(deviation_dict)
