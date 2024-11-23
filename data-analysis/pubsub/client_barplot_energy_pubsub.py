import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import glob

# Paths
prefix = "../../exp_runners/experiments/cpp_py_ros2_pub_sub"
s_folder = "../../exp_runners/experiments/cpp_py_ros2_pub_sub"
d_folder = "./graphs"

# Ensure the output folder exists
os.makedirs(d_folder, exist_ok=True)

# Load the runtable
df = pd.read_csv(f'{s_folder}/run_table.csv', nrows=160)

# Calculate average CPU power for each run_id
runs_data = {}
for run_id in df['__run_id']:
    folder_path = f"{s_folder}/{run_id}"
    server_energy_files = glob.glob(os.path.join(folder_path, 'energy-client-*.csv'))
    if server_energy_files:
        try:
            server_energy_df = pd.read_csv(server_energy_files[0])
            avg_server_energy_pct = server_energy_df['CPU Power'].mean()
            runs_data[run_id] = avg_server_energy_pct
        except Exception as e:
            print(f"Error processing file for run_id {run_id}: {e}")
    else:
        print(f"No energy-server files found for run_id {run_id}")

# Create a DataFrame for average CPU power and merge with the main DataFrame
avg_server_energy_df = pd.DataFrame(list(runs_data.items()), columns=['__run_id', 'avg_server_energy_pct'])
df = df.merge(avg_server_energy_df, on='__run_id')

# Group data and calculate the sum of CPU power
grouped_data = df.groupby(['msg_interval', 'language', 'num_clients'])['avg_server_energy_pct'].mean().reset_index()

# Create the barplot
plt.figure(figsize=(12, 8))
sns.barplot(
    data=grouped_data,
    x='msg_interval',
    y='avg_server_energy_pct',
    hue='language',
    palette='viridis',
    ci=None,
    dodge=True
)

# Customize the plot
plt.xlabel("Message Interval")
plt.ylabel("Power (W)")
plt.legend(title="Language")
plt.xticks(rotation=45)
plt.tight_layout()

# Save the plot
output_path = os.path.join(d_folder, 'client_energy_barplot.png')
plt.savefig(output_path)
plt.show()

print(f"Barplot saved to {output_path}")
