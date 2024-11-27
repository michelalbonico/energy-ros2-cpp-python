import sys
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import glob
# statistics
from scipy.stats import shapiro, kstest
from scipy.stats import levene
from scipy.stats import f_oneway
import statsmodels.api as sm
import statsmodels.formula.api as smf

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
        num_rows=250
    case 'action':
        pass

# Paths
prefix = f"../exp_runners/experiments/{algo_folder}"
s_folder = f"../exp_runners/experiments/{algo_folder}"
d_folder = f"./graphs/{dest_folder}"

# Ensure the output folder exists
os.makedirs(d_folder, exist_ok=True)

# Run Table
def load_run_table():
    global num_rows
    return pd.read_csv(f'{s_folder}/run_table.csv', nrows=num_rows)

def load_all_energy(component, msg_interval):
    df = load_run_table()
    # Avg CPU per Interval
    runs_data = {}
    energy_files = {}
    avg_cpu_df = {}
    for run_id in df['__run_id']:
        folder_path = f"{s_folder}/{run_id}"
        energy_files = glob.glob(os.path.join(folder_path, f'energy-{component}-*.csv'))
        if energy_files:
            try:
                energy_df = pd.read_csv(energy_files[0])
                energy_df['CPU Power'] = pd.to_numeric(energy_df['CPU Power'], errors='coerce')
                avg_energy_pct = energy_df['CPU Power'].mean()
                runs_data[run_id] = avg_energy_pct
            except Exception as e:
                print(f"Error processing file for run_id {run_id}: {e}")

    avg_cpu_df = pd.DataFrame(list(runs_data.items()), columns=['__run_id', 'avg_energy_pct'])
    df = df.merge(avg_cpu_df, on='__run_id')

    df = df.fillna(0)
    df = df[df["msg_interval"] == msg_interval]

    return df

#####################
# Statistical Tests #
#####################

components = {'server', 'client'}
intervals = {0.05, 0.25, 0.5, 1.0}

for component in components:
    for interval in intervals:
        print(f"------------ {component} ---------- {interval} ------------")
        df = load_all_energy(component,interval)

        # grouped_df = df.groupby(['language', 'num_clients'])
        grouped_df = df.groupby(['language', 'num_clients'])

        # Shapiro-Wilk
        print("# Shapiro-Wilk Test #")
        for (language, num_clients), group in grouped_df:
            stat, p_value = shapiro(group['avg_energy_pct'])
            print(f"Shapiro-Wilk test for {language}, {num_clients} clients: p = {p_value}")
        
        # Boxplots
        plt.figure(figsize=(3, 3))  # Create a new figure with a square size (6x6 inches)
        sns.boxplot(data=df, x='language', y='avg_energy_pct', hue='num_clients')
        plt.xlabel("Language")
        plt.ylabel("Power (W)")
        plt.tight_layout()  # Adjust layout to avoid overlap
        output_path = os.path.join(d_folder, f'{algo}_{component}_energy_boxplot_{interval}.pdf')
        plt.savefig(output_path, format="pdf", dpi=300)
        plt.close()

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
        # for (language, num_clients), group in df.groupby(['language', 'num_clients']):
        #     sns.kdeplot(
        #         data=group['avg_energy_pct'], 
        #         label=f"{language}, {num_clients} clients", 
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
        print("# Lavene Test - Language #")
        grouped_data = [group['avg_energy_pct'] for _, group in df.groupby('language')]
        stat, p_value = levene(*grouped_data)
        print(f"Levene's test: p = {p_value}")

        # # # ANOVA (one way)
        print("# ANOVA - Number of Clients")
        groups = [group['avg_energy_pct'] for _, group in df.groupby('num_clients')]
        stat, p_value = f_oneway(*groups)
        print(f"ANOVA test: p = {p_value}")

        print("-----------------------------------------------")

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
# model = smf.ols('avg_energy_pct ~ language * num_clients', data=df).fit()
# print(model.summary())
