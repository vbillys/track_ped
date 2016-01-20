from matplotlib import pyplot as plt

weights = [158.0, 164.2, 160.3, 159.9, 162.1, 164.6, 169.6, 167.4, 166.4, 171.0, 171.2, 172.6]

time_step = 1 # day
scale_factor = 4/10.

def predict_using_gain_guess(weight, gain_rate, do_print=True): 
	# store the filtered results
    estimates, predictions = [weight], []

    # most filter literature uses 'z' for measurements
    for z in weights: 
	    # predict new position
	prediction = weight + gain_rate * time_step

	# update filter 
	weight = prediction + scale_factor * (z - prediction)

	# save
	estimates.append(weight)
	predictions.append(prediction)
	print weight, estimates[-1], predictions[-1] 
	# if do_print:
		# gh.print_results(estimates, prediction, weight)

    # plot results
    # gh.plot_gh_results(weights, estimates, predictions)
    plt.plot(weights, '.g', label='Measurements')
    plt.plot(estimates, '-b', label='Estimates')
    plt.plot(predictions, '.r', label='Predictions')
    plt.legend(loc='lower right')
    plt.show()


initial_guess = 160.
predict_using_gain_guess(weight=initial_guess, gain_rate=1)
