#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


def fit(order = 2, plotter = False):
    ## Measured Irradiance values
    ## Iradiance value at .4 meters from UV sensor
    # ir = [17.69, 16.38, 14.36, 12.21, 11.68, 12.11, 11.76, 10.64, 8.19, 5.13, 3.50, 2.57, 1.96, 1.56, 1.24, 1.00,
    #        0.86,  0.68,  0.58,  0.49,  0.43,  0.39,  0.36,  0.34, 0.32, 0.31, 0.29, 0.28, 0.28, 0.27, 0.26, 0.25, 0.25]

    # Iradiance value at .3 meters from UV sensor. Trial #2 mW/cm^2
    ir = [26.96,25.27, 21.61, 17.78, 16.46, 16.57, 15.11, 11.30, 6.53, 3.71, 2.62, 1.93, 1.44, 1.09, 0.89, 0.75, 0.66,
          0.59, 0.55, 0.52, 0.50, 0.48, 0.46, 0.44, 0.43, 0.42, 0.40, 0.39, 0.38, 0.37, 0.35, 0.34, 0.32]

    # Meter positions from the center of the lit surface. stopped at 16cm
    x = np.linspace(0,(len(ir)-1)/2, len(ir))

    # Polyfit for data.
    model = np.poly1d(np.polyfit(x, ir, order))

    # Create a sequence of values to plug in the model
    x_line = np.linspace(0,(len(ir)-1)/2,len(ir))

    # Input of x_line of data
    best_fit = model(x_line)


    if plotter:
        # Plot both original data and best fit. Only plotted data that went up to
        # 10 cm.
        plt.grid(linestyle='--')
        # plt.scatter(x, ir)
        plt.plot(x_line[0:21], best_fit[0:21], '--', color = 'red', zorder=2)
        plt.errorbar(x[0:21], ir[0:21], xerr = .5, yerr = .4, capsize = 3,  fmt = 'o',zorder=1 )
        plt.fill_between(x[0:11], 0 ,best_fit[0:11], facecolor='green', alpha=0.3, zorder=3)
        plt.xlabel('Distance from the Center of Lit Surface $(cm)$', fontsize=14)
        plt.ylabel('Irradiance ($mW/cm^2$)',fontsize=14)
        plt.title('UV Flashlight at 30$cm$ above UV Meter',fontsize=16)
        plt.legend(['Best Fit Curve', 'Considered UV Irradation','UV Meter Measurements'], fontsize=12)
        # plt.subplots_adjust(left = 0.10, bottom = 0.09, right = 0.99, top =0.94)
        # plt.savefig("distance_vs_irradiance.png", bbox_inches='tight')
        plt.show()

    # This returns a polynomial function that takes single value or an array
    # as an input value. Ex: model(np.array([0,1,2,3]))
    return model

if __name__ == '__main__':
    model = fit(16, plotter = True)
    # print(model(2.5))
    # print(model(5.0))
    # print(model)
