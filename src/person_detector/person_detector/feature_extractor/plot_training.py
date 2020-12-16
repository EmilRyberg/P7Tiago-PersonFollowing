import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def plot_training(super_class_csv_file, sub_class_csv_file, overall_csv_file, out_file_name):
    pd_data_super = pd.read_csv(super_class_csv_file)
    pd_data_sub = pd.read_csv(sub_class_csv_file)
    pd_data_overall = pd.read_csv(overall_csv_file)

    fig = plt.figure(figsize=(14, 6))
    ax1 = fig.add_subplot(2, 2, 1, xlabel="Epoch", ylabel="Accuracy [%]", title="Super class", ylim=(20, 100), xlim=(1, 60))
    ax2 = fig.add_subplot(2, 2, 2, xlabel="Epoch", ylabel="Accuracy [%]", title="Sub class", ylim=(20, 100), xlim=(1, 60))
    ax3 = fig.add_subplot(2, 1, 2, xlabel="Epoch", ylabel="Accuracy [%]", title="Overall", ylim=(20, 100), xlim=(1, 60))
    ax1.plot(pd_data_super["Step"], pd_data_super["Value"])
    ax1.grid(True)
    ax2.plot(pd_data_sub["Step"], pd_data_sub["Value"])
    ax2.grid(True)
    ax3.plot(pd_data_overall["Step"], pd_data_overall["Value"])
    ax3.grid(True)

    plt.show()
    fig.savefig(out_file_name)


if __name__ == "__main__":
    plot_training("run-run_6_days_8_emb-tag-super_class_test_accuracy.csv",
                  "run-run_6_days_8_emb-tag-sub_class_test_accuracy.csv",
                  "run-run_6_days_8_emb-tag-overall_test_accuracy.csv", "plot.png")
