import statistics
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import warnings

def fit_best_distribution(data):
    distributions = [stats.norm, stats.expon, stats.gamma, stats.beta, stats.lognorm]
    best_fit_distribution = None
    best_fit_params = None
    best_aic = np.inf

    for distribution in distributions:
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore')
            try:
                params = distribution.fit(data)
                # Compute the log-likelihood
                log_likelihood = np.sum(distribution.logpdf(data, *params))
                # Compute the AIC
                aic = 2 * len(params) - 2 * log_likelihood

                if aic < best_aic:
                    best_fit_distribution = distribution
                    best_fit_params = params
                    best_aic = aic
            except Exception as e:
                continue

    return best_fit_distribution, best_fit_params


def calculate_statistics(filename):
    """
    This function calculates various statistical values for categories A and B from a file.

    Args:
        filename: The name of the file containing the data.

    Returns:
        A dictionary containing statistical values for A and B, or None if there's an error.
    """
    try:
        with open(filename, 'r') as file:
            data_a = []
            data_b = []
            for line in file:
                data = line.strip().split(':')
                if len(data) != 2:
                    print(f"Error: Invalid format in line: {line}")
                    return None
                if data[0] == 'A':
                    data_a.append(int(data[1]) / 10000.0)
                elif data[0] == 'B':
                    data_b.append(int(data[1]) / 10000.0)
                else:
                    print(f"Error: Unknown category: {data[0]}")
                    return None

            if not data_a:
                print("Warning: No data found for category A")
            if not data_b:
                print("Warning: No data found for category B")

            # Step 4: Fit the best distribution
            best_distribution, best_params = fit_best_distribution(data_b)

            # Step 5: Plot the histogram of the data
            plt.hist(data_b, bins=30, density=True, alpha=0.6, color='g', label='Histograma')

            # Plot the best-fitting distribution
            if best_distribution:
                xmin, xmax = plt.xlim()
                x = np.linspace(xmin, xmax, 100)
                p = best_distribution.pdf(x, *best_params)
                plt.plot(x, p, 'k', linewidth=2, label=f'Distribución para ajuste: {best_distribution.name}')

            # Add legend and labels
            plt.legend()
            plt.xlabel('Distancia(m)')
            plt.ylabel('Densidad')
            plt.title('Distribución de distancias obtenidas mediante SS-TWR')
            
            plt.savefig('dist.eps', format='eps')
            
            # Show the plot
            plt.show()

            stats_a = {
                "count": len(data_a),
                "mean": statistics.mean(data_a) if data_a else None,
                "median": statistics.median(data_a) if data_a else None,
                "stdev": statistics.stdev(data_a) if data_a else None,
                "min_value": min(data_a) if data_a else None,
                "max_value": max(data_a) if data_a else None,
            }
            stats_b = {
                "count": len(data_b),
                "mean": statistics.mean(data_b) if data_b else None,
                "median": statistics.median(data_b) if data_b else None,
                "stdev": statistics.stdev(data_b) if data_b else None,
                "min_value": min(data_b) if data_b else None,
                "max_value": max(data_b) if data_b else None,
            }
            return stats_a, stats_b
    except FileNotFoundError:
        print(f"Error: File not found: {filename}")
        return None
    except ValueError:
        print(f"Error: Invalid data format in file")
        return None

# Replace "data.txt" with the actual filename
filename = "data_8m.txt"
stats_a, stats_b = calculate_statistics(filename)

if stats_a and stats_b:
    avg_a = stats_a['mean']
    avg_b = stats_b['mean']

    minimum = min(avg_a, avg_b)
    maximum = max(avg_a, avg_b)

    real_dist = 4.4

    dw_delay_offs = minimum - real_dist
    dw_delay_offs = ((dw_delay_offs / 299702547) * 1e9 * 63.897763) // 2

    dw_delay_diff = (((maximum - minimum) / 299702547) * 1e9 * 63.897763) // 2

    print(f"DW_delay offset: {dw_delay_offs}")
    print(f"DW_delay diff: {dw_delay_diff}")

    print("Statistics for A:")
    for key, value in stats_a.items():
        print(f"{key}: {value:.2f}") if isinstance(value, float) else print(f"{key}: {value}")
    print("\nStatistics for B:")
    for key, value in stats_b.items():
        print(f"{key}: {value:.2f}") if isinstance(value, float) else print(f"{key}: {value}")
else:
    print("Unable to calculate statistics.")
