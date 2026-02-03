import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def analyze_data(folder_path, file_name, has_telemetry, set_voltage, order, if_reverse, use_f_norm):
    file_path = folder_path + file_name
    column_names = []
    if not has_telemetry:
        column_names = ["PWM", "fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency", "State"]
    else:
        column_names = [
            "PWM",
            "fx",
            "fy",
            "fz",
            "f_norm",
            "mx",
            "my",
            "mz",
            "currency",
            "RPM",
            "temperature",
            "voltage",
            "State",
        ]

    data = pd.read_csv(file_path, sep=" ", names=column_names)

    # Filter out the rows where State is 'done'
    data = data[data["PWM"] != "done"]

    # convert PWM from string to int
    data["PWM"] = data["PWM"].astype(int)

    # Filter the DataFrame for 'valid' state
    valid_data = data[data["State"] == "valid"]

    if not has_telemetry:
        columns_to_average = ["fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency"]
    else:
        columns_to_average = ["fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency", "RPM", "temperature", "voltage"]

    if if_reverse:
        valid_data.loc[:, ["fz", "mz", "f_norm"]] *= -1

    # Group by 'PWM' and calculate the average for each specified column
    average_values = valid_data.groupby("PWM")[columns_to_average].mean()

    # Resetting index to make 'PWM' a column again
    average_values.reset_index(inplace=True)

    # Add a column for power
    if has_telemetry:
        average_values["Power"] = average_values["voltage"] * average_values["currency"]
        print("Average voltage: {}".format(average_values["voltage"].mean()))
    else:
        average_values["Power"] = set_voltage * average_values["currency"]

    # Add a column for PWM ratio
    average_values["PWM_Ratio_%"] = average_values["PWM"] / 2000 * 100

    if has_telemetry:
        # Add a column for kRPM^2 (kilorevolutions per minute squared)
        average_values["kRPM^2"] = (average_values["RPM"] / 1000) ** 2
        # Add a column for Dshot ratio in percentage
        average_values["Dshot_Ratio_%"] = ((average_values["PWM"] - 1000) / 1000) * 100

    """ plot the data """

    # Convert series to numpy arrays

    if has_telemetry:
        kRPM2 = average_values["kRPM^2"].to_numpy()
        kRPM = average_values["RPM"].to_numpy() / 1000

    # Select force and moment based on use_f_norm parameter
    if use_f_norm:
        fz = average_values["f_norm"].to_numpy()
        force_label = "f_norm"
    else:
        fz = average_values["fz"].to_numpy()
        force_label = "fz"

    # mz always uses the original value
    mz = average_values["mz"].to_numpy()
    moment_label = "mz"

    currency = average_values["currency"].to_numpy()
    PWM_ratio = average_values["PWM_Ratio_%"].to_numpy()

    # Perform the fittings
    if has_telemetry:
        # Linear fit for x:kRPM^2 y:fz
        slope_kRPM2, intercept_kRPM2 = np.polyfit(kRPM2, fz, 1)
        fit_eq_kRPM2 = "{} = {:.4f} * kRPM^2 + {:.4f}".format(force_label, slope_kRPM2, intercept_kRPM2)
        print("Fitting Equation (x:kRPM^2 y:{}): {}".format(force_label, fit_eq_kRPM2))

        # Linear fit for x: pwm_ratio y: kRPM
        slope_kRPM_PWM_ratio, intercept_kRPM_PWM_ratio = np.polyfit(PWM_ratio, kRPM, 1)
        fit_eq_kRPM_PWM_ratio = "kRPM = {:.4f} * PWM_Ratio_% + {:.4f}".format(
            slope_kRPM_PWM_ratio, intercept_kRPM_PWM_ratio
        )
        print("Fitting Equation (x:PWM_Ratio_% y:kRPM): {}".format(fit_eq_kRPM_PWM_ratio))

    # Linear fit for x:fz y:mz, note that the intercept must be 0. Only y=kx form.
    slope_mz_fz, intercept_mz_fz = np.polyfit(fz, mz, 1)
    fit_eq_mz_fz = "{} = {:.4f} * {}".format(moment_label, slope_mz_fz, force_label)
    print("Fitting Equation (x:{} y:{}): {}".format(force_label, moment_label, fit_eq_mz_fz))

    # 2nd-order polynomial fit for x:fz y:currency
    coeffs_currency_fz = np.polyfit(fz, currency, 2)
    fit_eq_currency_fz = "currency = {:.4f} * {}^2 + {:.4f} * {} + {:.4f}".format(
        coeffs_currency_fz[0], force_label, coeffs_currency_fz[1], force_label, coeffs_currency_fz[2]
    )
    print("Fitting Equation (x:{} y:currency): {}".format(force_label, fit_eq_currency_fz))

    # n-th order polynomial fit for x:PWM_ratio y:fz
    coeffs_PWM_ratio_fz = np.polyfit(PWM_ratio, fz, order)
    fit_eq_PWM_ratio_fz = "{} = ".format(force_label)
    for i in range(order + 1):
        fit_eq_PWM_ratio_fz += "\n{:.4e} * PWM_Ratio_%^{} +".format(coeffs_PWM_ratio_fz[i], order - i)
    print()
    print("Fitting Equation (x:PWM_Ratio_% y:{}): {}".format(force_label, fit_eq_PWM_ratio_fz))
    for i in range(order + 1):
        print(
            "polynomial{}: {:.8f}  # x10^{}".format(order - i, pow(10, order - i) * coeffs_PWM_ratio_fz[i], order - i)
        )
    print()

    # n-th order polynomial fit for x:fz y:PWM_ratio
    coeffs_fz_PWM_ratio = np.polyfit(fz, PWM_ratio, order)
    fit_eq_fz_PWM_ratio = "PWM_ratio_% = "
    for i in range(order + 1):
        fit_eq_fz_PWM_ratio += "\n{:.4e} * {}^{} + ".format(coeffs_fz_PWM_ratio[i], force_label, order - i)
    print("Fitting Equation (x:{} y:PWM_Ratio_%): {}".format(force_label, fit_eq_fz_PWM_ratio))
    print("voltage: {}".format(set_voltage))
    print("max_thrust: {:.4f} # N".format(np.max(fz)))
    for i in range(order + 1):
        print(
            "polynomial{}: {:.8f}  # x10^{}".format(order - i, pow(10, order - i) * coeffs_fz_PWM_ratio[i], order - i)
        )
    print()

    # Create 3x3 subplots
    fig, axs = plt.subplots(3, 3, figsize=(12, 18))

    # Plot 1: x:kRPM^2 y:fz
    if has_telemetry:
        print("has telemetry", has_telemetry)
        axs[0, 0].scatter(kRPM2, fz, color="blue", alpha=0.5, label="Data points")
        axs[0, 0].plot(kRPM2, slope_kRPM2 * kRPM2 + intercept_kRPM2, color="red", label="Fitted line")
        axs[0, 0].set_title("{} vs kRPM^2".format(force_label))
        axs[0, 0].set_xlabel("kRPM^2")
        axs[0, 0].set_ylabel("{} (N)".format(force_label))
        axs[0, 0].grid()
        axs[0, 0].text(
            0.95,
            0.05,
            fit_eq_kRPM2,
            horizontalalignment="right",
            verticalalignment="bottom",
            transform=axs[0, 0].transAxes,
            fontsize=10,
            color="green",
        )

    # Plot 2: x:fz y:mz
    axs[0, 1].scatter(fz, mz, color="blue", alpha=0.5, label="Data points")
    axs[0, 1].plot(fz, slope_mz_fz * fz, color="red", label="Fitted line")
    axs[0, 1].set_title("{} vs {}".format(moment_label, force_label))
    axs[0, 1].set_xlabel("{} (N)".format(force_label))
    axs[0, 1].set_ylabel("{} (N*m)".format(moment_label))
    axs[0, 1].grid()
    axs[0, 1].text(
        0.95,
        0.05,
        fit_eq_mz_fz,
        horizontalalignment="right",
        verticalalignment="bottom",
        transform=axs[0, 1].transAxes,
        fontsize=10,
        color="green",
    )

    # Plot 3: x:fz y:currency
    axs[1, 0].scatter(fz, currency, color="blue", alpha=0.5, label="Data points")
    axs[1, 0].plot(np.sort(fz), np.polyval(coeffs_currency_fz, np.sort(fz)), color="red", label="Fitted polynomial")
    axs[1, 0].set_title("currency vs {}".format(force_label))
    axs[1, 0].set_xlabel("{} (N)".format(force_label))
    axs[1, 0].set_ylabel("currency")
    axs[1, 0].grid()
    axs[1, 0].text(
        0.95,
        0.05,
        fit_eq_currency_fz,
        horizontalalignment="right",
        verticalalignment="bottom",
        transform=axs[1, 0].transAxes,
        fontsize=10,
        color="green",
    )

    # Plot 4: fz vs PWM_ratio
    axs[1, 1].scatter(PWM_ratio, fz, color="blue", alpha=0.5, label="Data points")
    axs[1, 1].plot(
        np.sort(PWM_ratio), np.polyval(coeffs_PWM_ratio_fz, np.sort(PWM_ratio)), color="red", label="Fitted polynomial"
    )
    axs[1, 1].set_title("{} vs PWM_Ratio_%".format(force_label))
    axs[1, 1].set_xlabel("PWM_Ratio_%")
    axs[1, 1].set_ylabel("{} (N)".format(force_label))
    axs[1, 1].grid()
    axs[1, 1].text(
        0.95,
        0.05,
        fit_eq_PWM_ratio_fz,
        horizontalalignment="right",
        verticalalignment="bottom",
        transform=axs[1, 1].transAxes,
        fontsize=10,
        color="green",
    )

    # Plot 5: fz vs PWM_ratio^2
    axs[2, 0].scatter(PWM_ratio**2, fz, color="blue", alpha=0.5, label="Data points")
    axs[2, 0].set_title("{} vs PWM_Ratio_%^2".format(force_label))
    axs[2, 0].set_xlabel("PWM_Ratio_%^2")
    axs[2, 0].set_ylabel("{} (N)".format(force_label))
    axs[2, 0].grid()

    # Plot 6: kRPM vs PWM_ratio
    if has_telemetry:
        axs[2, 1].scatter(PWM_ratio, kRPM, color="blue", alpha=0.5, label="Data points")
        axs[2, 1].plot(
            PWM_ratio, slope_kRPM_PWM_ratio * PWM_ratio + intercept_kRPM_PWM_ratio, color="red", label="Fitted line"
        )
        axs[2, 1].set_title("kRPM vs PWM_Ratio_%")
        axs[2, 1].set_xlabel("PWM_Ratio_%")
        axs[2, 1].set_ylabel("kRPM")
        axs[2, 1].grid()
        axs[2, 1].text(
            0.95,
            0.05,
            fit_eq_kRPM_PWM_ratio,
            horizontalalignment="right",
            verticalalignment="bottom",
            transform=axs[2, 1].transAxes,
            fontsize=10,
            color="green",
        )

    # Plot 7: PWM_ratio vs fz
    axs[0, 2].scatter(fz, PWM_ratio, color="blue", alpha=0.5, label="Data points")
    axs[0, 2].plot(np.sort(fz), np.polyval(coeffs_fz_PWM_ratio, np.sort(fz)), color="red", label="Fitted polynomial")
    axs[0, 2].set_title("PWM_Ratio_% vs {}".format(force_label))
    axs[0, 2].set_xlabel("{} (N)".format(force_label))
    axs[0, 2].set_ylabel("PWM_Ratio_%")
    axs[0, 2].grid()
    axs[0, 2].text(
        0.95,
        0.05,
        fit_eq_fz_PWM_ratio,
        horizontalalignment="right",
        verticalalignment="bottom",
        transform=axs[0, 2].transAxes,
        fontsize=10,
        color="green",
    )

    # title with the file name
    fig.suptitle(file_name)

    # Adjust layout and show plot
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # pass text file name as argument
    parser = argparse.ArgumentParser()
    parser.add_argument("file_name", help="file name")
    parser.add_argument("--has_telemetry", "-t", help="has telemetry", action="store_true")
    parser.add_argument("--set_voltage", "-s", help="the voltage set in the test, float", default=0.0)
    parser.add_argument("--order", "-o", help="fitting order of fz vs PWM_ratio", default=2)
    parser.add_argument("--folder_path", "-f", help="path to folder", default="~/.ros/")
    parser.add_argument("--if_reverse", "-r", help="if the force sensor is reversed", action="store_true")
    parser.add_argument("--use_f_norm", "-n", help="use f_norm instead of fz for force", action="store_true")
    args = parser.parse_args()

    analyze_data(
        args.folder_path,
        args.file_name,
        args.has_telemetry,
        float(args.set_voltage),
        int(args.order),
        args.if_reverse,
        args.use_f_norm,
    )
