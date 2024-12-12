import os
import pandas as pd
import matplotlib.pyplot as plt
import fnmatch
import seaborn as sns

from statistical_tests import shapiro_wilk

def get_average_cpu_power(folder_path, file_pattern):
    averages = []

    for root, _, files in os.walk(folder_path):
        matching_files = fnmatch.filter(files, file_pattern)  # Match files using pattern
        for filename in matching_files:
            csv_path = os.path.join(root, filename)
            try:
                df = pd.read_csv(csv_path)
                if 'CPU Power' in df.columns:
                    # Filter CPU Power values below 5 and calculate the mean
                    filtered_power = df['CPU Power'][df['CPU Power'] < 5].dropna()
                    avg_power = filtered_power.mean()
                    if not pd.isna(avg_power):
                        averages.append(avg_power)
            except Exception as e:
                pass
                #print(f"Error processing {csv_path}: {e}")

    return averages

def dict_to_df(data_dict):
    all_data = []
    labels = []
    for label, values in data_dict.items():
        all_data.extend(values)
        labels.extend([label] * len(values))

    df = pd.DataFrame({
        'Folder': labels,
        'Average CPU Power': all_data
    })

    return df

def plot_boxplots(data_dict):
    plt.figure(figsize=(10, 6))

    # Extract data and labels
    data = list(data_dict.values())
    labels = list(data_dict.keys())

    plt.boxplot(data, labels=labels)
    plt.ylabel('Average CPU Power')
    plt.title('Boxplot of Average CPU Power for Different Folders')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.show()

def plot_violin(data_dict, component):
    plt.figure(figsize=(10, 6))

    # Prepare data for Seaborn violin plot
    # all_data = []
    # labels = []
    # for label, values in data_dict.items():
    #     all_data.extend(values)
    #     labels.extend([label] * len(values))

    # violin_data = pd.DataFrame({
    #     'Folder': labels,
    #     'Average CPU Power': all_data
    # })

    violin_data = dict_to_df(data_dict)

    plt.figure(figsize=(6, 3))

    sns.violinplot(x='Folder', y='Average CPU Power', data=violin_data, inner='box')

    if component == 'server':
        custom_labels = ['publisher', 'service server', 'action server']
    else:
        custom_labels = ['subscriber', 'service client', 'action client']
    plt.xticks(ticks=range(len(custom_labels)), labels=custom_labels, rotation=15, ha="right")
    plt.xlabel('')

    plt.ylabel('CPU Power')

    plt.tight_layout()
    #plt.show()

    output_file_name = f"overall_{component}_power_violin.pdf"
    output_path = os.path.join('graphs', output_file_name)
    plt.savefig(output_path, format="pdf", dpi=300)
    plt.close()


def main():
    folders = [
        '../exp_runners/experiments/cpp_py_ros2_pub_sub',
        '../exp_runners/experiments/cpp_py_ros2_service',
        '../exp_runners/experiments/cpp_py_ros2_action'
    ]

    components = {'server','client'}

    for component in components:
        data_dict = {}

        file_pattern = f'energy-{component}-*.csv'

        for folder in folders:
            folder_name = os.path.basename(folder.strip('/'))
            averages = get_average_cpu_power(folder,file_pattern)
            data_dict[folder_name] = averages
        
        df = dict_to_df(data_dict)
        grouped_df = df.groupby('Folder')

        shapiro_wilk(grouped_df,'Average CPU Power','Folder')

        #plot_violin(data_dict, component)

if __name__ == "__main__":
    main()