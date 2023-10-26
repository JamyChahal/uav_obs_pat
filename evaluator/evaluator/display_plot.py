import os

import numpy as np
import pandas as pd

pd.set_option('display.max_rows', None)
import seaborn as sns
from matplotlib import pyplot as plt

dir_result = 'result_test'
dir_result = 'result_smallcluster_old'
dir_result = 'result_cluster2'
time_min = 5
time_max = 3600 * 3

def plot(df_full, metrics, methods, folder_name):
    #directory = os.path.dirname(os.path.realpath(__file__)) + '/' + str(dir_result) + '/'
    directory = os.path.dirname(os.path.realpath(__file__)) + '/' + str(dir_result) + '/plots/'
    metric = metrics[0]
    colors = iter(['green', 'blue', 'red', 'black', 'yellow', 'purple', 'darkblue', 'steelblue'])

    sns.set()
    for m in methods:
        if not 'I-CMOMMT' in m:
            continue
        # Time filtering
        df_full[m] = df_full[m].query('time > ' + str(time_min) + ' & time < ' + str(time_max))
        x_axis = df_full[m]['time'].unique()
        x_axis = sorted(x_axis)
        color = next(colors)

        metric_mean_df = df_full[m].groupby('time', as_index=False)[metric].mean()
        metric_max_df = df_full[m].groupby('time', as_index=False)[metric].max()
        metric_min_df = df_full[m].groupby('time', as_index=False)[metric].min()

        values = metric_mean_df[metric]

        plt.plot(x_axis, values, color, label=m)
        max_values = metric_max_df[metric]
        min_values = metric_min_df[metric]
        plt.fill_between(x_axis, min_values, max_values,
                         color=color, alpha=0.1)

    plt.xlabel('Temps (s)')
    plt.ylabel(metrics[1])
    plt.legend(loc='upper left')
    if not os.path.isdir(directory + folder_name):
        os.makedirs(directory + folder_name)
    plt.savefig(directory + folder_name + metrics[1], dpi=100)
    plt.clf()

def read_filenames_for_plot(filenames='', category='', methods=''):

    unique_methods = np.unique(np.array(methods))

    dir_name = ''
    for c in category:
        dir_name += str(c) + "=" + str(category[c]) + ";"
    dir_name += "/"


    method_df = {}
    df_full = {}
    for u in unique_methods:
        method_df[u] = []

    for i, filename in enumerate(filenames):
        print(filename)
        df = pd.read_csv(str(dir_result) + '/raw_data/' + str(filename), index_col=None, header=0, sep=";")
        method_df[methods[i]].append(df)

    for u in unique_methods:
        df_full[u] = pd.concat(method_df[u], axis=0, ignore_index=True)

    metrics_name = [('A_metric', 'Métrique A'),
                    ('H_metric', 'Métrique H'),
                    ('sigma_n', 'Déviation standard observation'),
                    ('I_oav_m', 'Oisiveté moyenne'),
                    ('I_inst', 'Oisiveté moyenne instantanée'),
                    ('max_idleness', 'Oisiveté maximale'),
                    ('max_idleness_region', 'Oisiveté maximale régionale'),
                    ('MI', 'MI'),
                    ('MSI', 'MSI')
                    ]

    for m in metrics_name:
        plot(df_full=df_full, metrics=m, methods=unique_methods, folder_name=dir_name)




if __name__ == "__main__":

    plt.rcParams['figure.figsize'] = (10, 5)

    df = pd.read_csv(str(dir_result)+'/summary_exp_filenames.txt', sep=';')

    column_names = ['nbr_agent', 'nbr_target', 'obs_range', 'com_range', 'map_size_x', 'map_size_y']

    #print(df.head())

    categories = df[column_names].value_counts().reset_index(name='count')

    nbr_categories = len(categories.index)

    for i in range(0, nbr_categories):
        df_fetch = df.loc[(df['nbr_agent'] == categories['nbr_agent'].iloc[i]) \
                          & (df['nbr_target'] == categories['nbr_target'].iloc[i]) \
                          & (df['obs_range'] == categories['obs_range'].iloc[i]) \
                          & (df['com_range'] == categories['com_range'].iloc[i]) \
                          & (df['map_size_x'] == categories['map_size_x'].iloc[i]) \
                          & (df['map_size_y'] == categories['map_size_y'].iloc[i])]
        print("Reading file..")
        filenames = df_fetch['filename'].to_numpy()
        methods = df_fetch['method'].to_numpy()
        categories_names = {}
        for c in column_names:
            categories_names[c] = df_fetch[c].iloc[0]
        read_filenames_for_plot(filenames, categories_names, methods)
