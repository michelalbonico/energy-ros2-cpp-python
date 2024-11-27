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

# Avg CPU
def load_avg_cpu(component):
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
                avg_cpu_pct = energy_df['CPU Utilization'].mean() * 100 * 8
                runs_data[run_id] = avg_cpu_pct
            except Exception as e:
                print(f"Error processing file for run_id {run_id}: {e}")

    avg_cpu_df = pd.DataFrame(list(runs_data.items()), columns=['__run_id', 'avg_cpu_pct'])
    df = df.merge(avg_cpu_df, on='__run_id')

    return df.groupby(['msg_interval', 'language', 'num_clients'])['avg_cpu_pct'].mean().reset_index()

def load_avg_memory(component):
    df = load_run_table()
    # Avg CPU per Interval
    runs_data = {}
    energy_files = {}
    avg_cpu_df = {}
    for run_id in df['__run_id']:
        folder_path = f"{s_folder}/{run_id}"
        energy_files = glob.glob(os.path.join(folder_path, f'cpu-mem-{component}.csv'))
        if energy_files:
            try:
                energy_df = pd.read_csv(energy_files[0])
                avg_mem = energy_df['memory_usage'].mean() / 1024
                runs_data[run_id] = avg_mem
            except Exception as e:
                print(f"Error processing file for run_id {run_id}: {e}")

    avg_cpu_df = pd.DataFrame(list(runs_data.items()), columns=['__run_id', 'avg_mem'])
    df = df.merge(avg_cpu_df, on='__run_id')

    return df.groupby(['msg_interval', 'language', 'num_clients'])['avg_mem'].mean().reset_index()

def load_avg_energy(component, msg_interval):
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

    return df.groupby(['language', 'num_clients'])

def load_total_energy(component, msg_interval):
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
                sum_energy_pct = energy_df['CPU Power'].sum()
                runs_data[run_id] = sum_energy_pct
            except Exception as e:
                print(f"Error processing file for run_id {run_id}: {e}")

    avg_cpu_df = pd.DataFrame(list(runs_data.items()), columns=['__run_id', 'sum_energy_pct'])
    df = df.merge(avg_cpu_df, on='__run_id')

    df = df.fillna(0)
    df = df[df["msg_interval"] == msg_interval]

    return df.groupby(['language', 'num_clients'])

def print_stats(grouped_data):
    for (language, num_clients), group in grouped_data:
        print(f"Language: {language}, Number of Clients: {num_clients}")
        print(group)
        print("-" * 50)  # Separator for readability

        group_mean = group.mean(numeric_only=True)
        print("Mean values for this group:")
        print(group_mean)
        print("=" * 50)  # Another separator for clarity 

##############
# Graphs Gen #
##############

# Barplot
def gen_cpu_barplot(component):
    
    grouped_data = load_avg_cpu(component)

    plt.figure(figsize=(10, 8))
    sns.barplot(
        data=grouped_data,
        x='msg_interval',
        y='avg_cpu_pct',
        hue='language',
        palette='viridis',
        errorbar=None,
        dodge=True
    )

    # Customize the plot
    plt.xlabel("Message Interval")
    plt.ylabel("Average CPU Usage (% of 1 core)")
    plt.xticks(rotation=45)
    plt.tight_layout()
    output_path = os.path.join(d_folder, f'{algo}_{component}_cpu_barplot.pdf')
    plt.savefig(output_path, format="pdf", dpi=300)

# Boxplots
def gen_energy_boxplot(component, msg_interval):
    grouped = load_avg_energy(component, msg_interval)

    boxplot_data = []
    labels = []

    for group_name, group_data in grouped:
        boxplot_data.append(group_data['avg_energy_pct'].values)
        labels.append(f"{group_name[0]}, {group_name[1]}")

    # Plot boxplot
    plt.figure(figsize=(10, 6))
    plt.boxplot(boxplot_data, labels=labels)
    plt.title(f"Interval: {msg_interval}")
    plt.ylabel("Average Power (W)")
    plt.xlabel("Language, Clients")
    plt.xticks(rotation=45)
    plt.tight_layout()

    output_file = os.path.join(d_folder, f"boxplot_{algo}_{component}_energy_{msg_interval}.pdf")
    plt.savefig(output_file, format="pdf", dpi=300)

components = {'client', 'server'}
intervals = {0.05, 0.25, 0.5, 1.0}

#for component in components:
#    gen_cpu_barplot(component)
#    for interval in intervals:
#        gen_energy_boxplot(component, interval)


#grouped_data = load_total_energy('server',0.05)
#grouped_data = load_avg_cpu('server')
# grouped_data = load_avg_memory('server')
# print(grouped_data)
# grouped_data = grouped_data[grouped_data["msg_interval"] == 0.05]
# print_stats(grouped_data)
