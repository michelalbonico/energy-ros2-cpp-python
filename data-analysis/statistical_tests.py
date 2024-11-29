import sys
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import glob
import numpy as np
# statistics
from scipy.stats import shapiro, kstest
from scipy.stats import levene
from scipy.stats import f_oneway
import statsmodels.api as sm
import statsmodels.formula.api as smf
from statsmodels.stats.anova import anova_lm
from scipy.stats import boxcox
from scipy.stats import yeojohnson

from load_data import LoadData


# Arguments
n = len(sys.argv)
if n <=1:
    print("Please, pass the algorithm name as an argument.")
    sys.exit(0)

algo=sys.argv[1]
algo_folder=''
dest_folder=''
num_rows=0

match algo:
    case 'pubsub':
        algo_folder='cpp_py_ros2_pub_sub'
        dest_folder='pubsub'
        num_rows=480
    case 'service':
        algo_folder='cpp_py_ros2_service'
        dest_folder='service'
        num_rows=112
    case 'action':
        pass

# Paths
prefix = f"../exp_runners/experiments/{algo_folder}"
s_folder = f"../exp_runners/experiments/{algo_folder}"
d_folder = f"./graphs/{dest_folder}"

# Ensure the output folder exists
os.makedirs(d_folder, exist_ok=True)

# Load Data Class
l_data = LoadData(num_rows, s_folder)
l_run_table = l_data.load_run_table()

# def load_all_energy(component, msg_interval):
#     #df = load_run_table()
#     df = l_run_table
#     # Avg CPU per Interval
#     runs_data = {}
#     energy_files = {}
#     avg_cpu_df = {}
#     for run_id in df['__run_id']:
#         folder_path = f"{s_folder}/{run_id}"
#         energy_files = glob.glob(os.path.join(folder_path, f'energy-{component}-*.csv'))
#         if energy_files:
#             try:
#                 energy_df = pd.read_csv(energy_files[0])
#                 energy_df['CPU Power'] = pd.to_numeric(energy_df['CPU Power'], errors='coerce')
#                 # Apply log transformation (log + 1 to avoid log(0) issues)
#                 # energy_df['CPU Power'] = energy_df['CPU Power'].apply(lambda x: np.log(x + 1) if x > 0 else np.nan)
#                 # Apply Box-Cox transformation (ensure the values are positive)
#                 # energy_df['CPU Power'], _ = boxcox(energy_df['CPU Power'].dropna() + 1)  # Adding 1 if data contains zeros or negatives
#                 avg_energy_pct = energy_df['CPU Power'].mean()
#                 runs_data[run_id] = avg_energy_pct
#             except Exception as e:
#                 print(f"Error processing file for run_id {run_id}: {e}")

#     avg_cpu_df = pd.DataFrame(list(runs_data.items()), columns=['__run_id', 'avg_energy_pct'])
#     df = df.merge(avg_cpu_df, on='__run_id')

#     # Log transformation
#     #df['log_avg_energy_pct'] = np.log(df['avg_energy_pct'].replace(0, np.nan))  # Avoid log(0)

#     # Box-cox
#     # df['avg_energy_pct'] = df['avg_energy_pct'].apply(lambda x: x + 1 if x <= 0 else x)
#     # df['boxcox_avg_energy_pct'], best_lambda = boxcox(df['avg_energy_pct'])

#     # Apply Yeo-Johnson Transformation (works for negative values as well)
#     # df['yeojohnson_avg_energy_pct'], best_lambda = yeojohnson(df['avg_energy_pct'])

#     df = df.fillna(0)
#     #df = df[df["msg_interval"] == msg_interval]
#     #df = df[df["num_clients"] == msg_interval]

#     return df

#####################
# Statistical Tests #
#####################

def shapiro_wilk(df_group):
    for (num_clients), group in grouped_df:
        stat, p_value = shapiro(group[df_group])
        print(f"Shapiro-Wilk test for {num_clients} clients: p = {p_value}")

def levene_test(grouped_df, value):
    group_values = [group[value].values for _, group in grouped_df]
    stat, p = levene(*group_values)
    print(f"Levene's test statistic: {stat}, p-value: {p}")

# Factors
#components = {'server', 'client'}
components = {'server'}
intervals = {0.05,0.25,0.5,1.0}
num_clients = {1,2,3}
languages = {'py','cpp'}

### FOR energy / CPU / ETC
for component in components:
    for language in languages:
        df = l_data.load_power(component, False)
        df_language = l_data.filter_df(df,'language',language)
        # for interval in intervals:
        #     df = df_language
        #     df = l_data.filter_df(df,'msg_interval',interval)
        #     print(f"Component: {component}, Interval: {interval}, Language: {language} --->")
            
        #     grouped_df = l_data.group_df_by(df,'num_clients')

        #     # Shapiro-Wilk
        #     shapiro_wilk('avg_energy_pct')

        #     # Levene's
        #     levene_test(grouped_df,'avg_energy_pct')

        #     # ANOVA (one way)
        #     groups = [group['avg_energy_pct'] for _, group in grouped_df]
        #     stat, p_value = f_oneway(*groups)
        #     print(f"ANOVA test: p = {p_value}")
            
        for clients in num_clients:
            df = df_language
            df = l_data.filter_df(df,'num_clients',clients)
            print(f"Component: {component}, Num clients: {clients}, Language: {language} --->")

            grouped_df = l_data.group_df_by(df,'msg_interval')

            # Shapiro-Wilk
            shapiro_wilk('avg_energy_pct')

            # Levene's
            levene_test(grouped_df,'avg_energy_pct')

    # Check Intervals

        # #print(f"------------ {component} ---------- {interval} ------------")
        # #df = load_all_energy(component,interval)
        # df = load_all_energy(component)
        # df = df[df["msg_interval"] == 0.05]

        # # parametric
        # # df = df[(df["num_clients"] == 1) | (df["num_clients"] == 2)]

        # # grouped_df = df.groupby(['language', 'num_clients'])
        # # grouped_df = df.groupby(['language', 'num_clients'])
        # #grouped_df = df.groupby(['language', 'msg_interval', 'num_clients'])
        # #grouped_df = df.groupby('num_clients')
        # grouped_df = df.groupby('msg_interval')
        # #print(grouped_df.groups)

        # Shapiro-Wilk
        # print("# Shapiro-Wilk Test #")
        #for (language, msg_interval), group in grouped_df:
        #for (language, num_clients), group in grouped_df:
        # for (num_clients), group in grouped_df:
        # #for (language), group in grouped_df:
        # #for (language, msg_interval, num_clients), group in grouped_df:
        #     stat, p_value = shapiro(group['avg_energy_pct'])
        #     #stat, p_value = shapiro(group['log_avg_energy_pct'])
        #     #stat, p_value = shapiro(group['boxcox_avg_energy_pct'])
        #     #stat, p_value = shapiro(group['yeojohnson_avg_energy_pct'])
        #     #print(f"Shapiro-Wilk test for {language}, {num_clients} clients: p = {p_value}")
        #     #print(f"Shapiro-Wilk test for {language}: p = {p_value}")
        #     #print(f"Shapiro-Wilk test for {language}, {msg_interval} interval: p = {p_value}")
        #     print(f"Shapiro-Wilk test: p = {p_value}")

        # print("# Shapiro-Wilk Test #")
        # for (language, msg_interval), group in grouped_df:
        #     stat, p_value = shapiro(group['avg_energy_pct'])
        #     print(f"Shapiro-Wilk test for {language}, {msg_interval} interval: p = {p_value}")
        
        # Boxplots
        # plt.figure(figsize=(3, 3))  # Create a new figure with a square size (6x6 inches)
        # sns.boxplot(data=df, x='language', y='avg_energy_pct', hue='num_clients')
        # plt.xlabel("Language")
        # plt.ylabel("Power (W)")
        # plt.tight_layout()  # Adjust layout to avoid overlap
        # output_path = os.path.join(d_folder, f'{algo}_{component}_energy_boxplot_{interval}.pdf')
        # plt.savefig(output_path, format="pdf", dpi=300)
        # plt.close()

        # # Plot histogram for each group
        # for (language, num_clients), group in df.groupby(['language', 'num_clients']):
        #     plt.hist(group['avg_energy_pct'], bins=10, alpha=0.7, label=f"{language}, {num_clients} clients")

        # plt.title("Histogram of avg_energy_pct")
        # plt.xlabel("avg_energy_pct")
        # plt.ylabel("Frequency")
        # plt.legend()
        # plt.show()

        # # Create a histogram with KDE line overlay for each group
        # for (language, num_clients), group in df.groupby(['language', 'num_clients']):
        #     sns.histplot(
        #         group['avg_energy_pct'], 
        #         bins=10, 
        #         kde=True, 
        #         stat="density",  # Normalize histogram to match KDE
        #         label=f"{language}, {num_clients} clients", 
        #         alpha=0.5  # Transparency for better overlay
        #     )

        # # Add labels, legend, and title
        # plt.title("Histogram and KDE of avg_energy_pct")
        # plt.xlabel("avg_energy_pct")
        # plt.ylabel("Density")
        # plt.legend()
        # plt.show()

        # Plot KDE lines with filled areas for each group
        #for (language, num_clients), group in df.groupby(['language', 'num_clients']):
        # for (language, msg_interval), group in df.groupby(['language', 'msg_interval']):
        #     sns.kdeplot(
        #         data=group['avg_energy_pct'], 
        #         #label=f"{language}, {num_clients} clients", 
        #         label=f"{language}, {msg_interval} interval", 
        #         fill=True,  # Fills the area under the line
        #         alpha=0.5,  # Transparency for the fill
        #         linewidth=2  # Thickness of the line
        #     )

        # # Add labels, legend, and title
        # plt.xlabel("Power")
        # plt.ylabel("Frequency")
        # plt.legend()
        # plt.show()

        # # Levene's
        # print("# Lavene Test - Only language #")
        # grouped_data = [group['avg_energy_pct'] for _, group in df.groupby('language')]
        # stat, p_value = levene(*grouped_data)
        # print(f"Levene's test: p = {p_value}")

        # print("# Lavene Test - Language and num_clients #")
        # grouped_data = [group['avg_energy_pct'] for _, group in df.groupby(['num_clients'])]
        # stat, p_value = levene(*grouped_data)
        # print(f"Levene's test: p = {p_value}")

        # print("# Lavene Test - Language and num_clients #")
        # grouped_data = [group['avg_energy_pct'] for _, group in df.groupby(['msg_interval'])]
        # stat, p_value = levene(*grouped_data)
        # print(f"Levene's test: p = {p_value}")

        # print("# Lavene Test - Language #")
        # grouped_data = [group['avg_energy_pct'] for _, group in df.groupby('language')]
        # stat, p_value = levene(*grouped_data)
        # print(f"Levene's test: p = {p_value}")

        # # # ANOVA (one way)
        # print("# ANOVA - Number of Clients")
        #groups = [group['avg_energy_pct'] for _, group in df.groupby('num_clients')]
        # groups = [group['avg_energy_pct'] for _, group in df.groupby('msg_interval')]
        # stat, p_value = f_oneway(*groups)
        # print(f"ANOVA test: p = {p_value}")

        # print("# two way ANOVA - Number of Clients")
        # groups = [group['avg_energy_pct'] for _, group in df.groupby('msg_interval')]
        # stat, p_value = anova_lm(*groups)
        # print(f"ANOVA test: p = {p_value}")

        # print("-----------------------------------------------")

# # Regression
# print("# Regression num_clients and msg_interval")
# df['num_clients'] = pd.to_numeric(df['num_clients'])
# df['msg_interval'] = pd.to_numeric(df['msg_interval'])

# X = df[['num_clients', 'msg_interval']]
# y = df['avg_energy_pct']
# X = sm.add_constant(X)  # Add intercept
# model = sm.OLS(y, X).fit()
# print(model.summary())

# # # Two-Way ANOVA
# print("#Two-way anova")
# model = smf.ols('avg_energy_pct ~ language * num_clients', data=df).fit()
# print(model.summary())