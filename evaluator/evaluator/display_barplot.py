import os

import numpy as np
import pandas as pd

pd.set_option('display.max_rows', None)
import seaborn as sns
from matplotlib import pyplot as plt
from scipy.signal import butter, filtfilt

dir_result = 'result'

if __name__ == "__main__":

    plt.rcParams['figure.figsize'] = (10, 5)
    CMOMMT_CHOICE = 'I-CMOMMT-0.5'

    remove_method = ['I-CMOMMT-0.1', 'I-CMOMMT-0.3', 'I-CMOMMT-0.5', 'I-CMOMMT-0.7', 'I-CMOMMT-0.9']
    remove_method.remove(CMOMMT_CHOICE)
    order_list = ['RANDOM', 'CI', 'HI', CMOMMT_CHOICE, 'A-CMOMMT', 'F2MARL']

    df = pd.read_csv(str(dir_result) + '/result.txt', sep=';')

    # Remove method
    df = df.drop(df[df.method.isin(remove_method)].index)
    # Remove where nbr target > nbr agent
    df = df.drop(df[df.nbr_agent > df.nbr_target].index)

    metrics = ['A_metric', 'H_metric', 'sigma_n', 'I_oav_m', 'max_idleness', 'max_idleness_region', 'MI', 'MSI']

    metrics_title = {'A_metric': 'Métrique A',
                     'H_metric': 'Métrique H',
                     'sigma_n': 'Déviation standard observation',
                     'I_oav_m': 'Oisiveté moyenne',
                     'max_idleness': 'Oisiveté maximale',
                     'max_idleness_region': 'Oisiveté maximale régionale',
                     'MI': 'MI',
                     'MSI': 'MSI'
                     }

    df["x_axis"] = "A" + df["nbr_agent"].astype(str) + "T" + df['nbr_target'].astype(str)

    directory = os.path.dirname(os.path.realpath(__file__)) + '/' + str(dir_result) + '/barplot/'
    if not os.path.isdir(directory):
        os.makedirs(directory)

    for metric in metrics:
        sns.set()
        df_metric = df[['x_axis', 'method', metric]]
        print("Generating plot for metric " + metric)

        ax = sns.barplot(
            data=df_metric,
            x='x_axis',
            y=metric,
            hue='method',
            errorbar='sd',  # Error as standard deviation
            hue_order=order_list
        )

        ax.set(xlabel='Configuration de n agents et m cibles (AnTm)', ylabel=metrics_title[metric])
        plt.legend(title='Méthode')
        #plt.show()
        plt.savefig(directory + metrics_title[metric], dpi=100)
        plt.clf()