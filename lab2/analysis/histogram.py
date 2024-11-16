#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def analyze_and_plot_deviations(file_path, plot_title):
    
    df = pd.read_csv(file_path)
    utm_easting = df['UTM_easting']
    utm_northing = df['UTM_northing']
    
    # Calculate mean positions
    mean_easting = utm_easting.mean()
    mean_northing = utm_northing.mean()

    deviation_easting = utm_easting - mean_easting
    deviation_northing = utm_northing - mean_northing
    average_deviation = np.mean(np.hypot(deviation_easting, deviation_northing))
    
    print(f"Mean position for {plot_title}: Easting={mean_easting:.2f}, Northing={mean_northing:.2f}")
    print(f"Mean deviation from the mean position for {plot_title}: {average_deviation:.2f} meters")
    
    # Create a 2D histogram of deviations
    plt.figure(figsize=(8, 6))
    plt.hist2d(deviation_easting, deviation_northing, bins=30, cmap='Blues')
    plt.colorbar(label='Frequency')
    plt.xlabel('Easting Deviation (m)')
    plt.ylabel('Northing Deviation (m)')
    plt.title(f'2D Histogram of Deviations - {plot_title}')
    plt.axhline(0, color='red', linestyle='dashed', linewidth=1)
    plt.axvline(0, color='red', linestyle='dashed', linewidth=1)
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # Paths to CSV files
    csv_file = [
        '/home/iftekhar/EECE5554/lab2/data/open_stationary/open_stationary.csv',
        '/home/iftekhar/EECE5554/lab2/data/occluded_stationary/occluded_stationary.csv',
        '/home/iftekhar/EECE5554/lab2/data/open_move/open_move.csv',
        '/home/iftekhar/EECE5554/lab2/data/occluded_move/occluded_move.csv' 
    ]
    
    # plots Title 
    plot_titles = ['Open Stationary', 'Occluded Stationary', 'Open Moving', 'Occluded Moving']
    
    for file_path, plot_title in zip(csv_file, plot_titles):
        analyze_and_plot_deviations(file_path, plot_title)
