import os
from load_data import LoadData
import pandas as pd

# Generate LaTeX table
def generate_latex_table(df):
    header = r"""
\begin{tabular}{l|p{2cm}p{2.2cm}p{2.2cm}p{2.2cm}p{2.2cm}p{2.2cm}}
\toprule
\rowcolor{gray}
\textbf{Language} & \textbf{\# Clients} & \textbf{Msg. Interval (s)} & \textbf{Avg. CPU Utilization (\%)} & \textbf{Avg. Memory Utilization (KB)} & \textbf{Total CPU Energy (J)} & \textbf{CPU Power (W)} \\
\midrule
"""
    body = ""
    for language, group in df.groupby("Language"):
        body += r"\multirow{3}{*}{\emph{" + language + "}} \n"
        for client in group["# Clients"].unique():
            rows = group[group["# Clients"] == client]
            first_line = True
            for _, row in rows.iterrows():
                if first_line:
                    multirow =  '\multirow{5}{*}{'
                    postmulti = '}'
                    clients = f"${row['# Clients']}$"
                    first_line=False
                else:
                    multirow = ''
                    postmulti = ''
                    clients = ''
                body += "&" + multirow + clients + postmulti +f"& ${row['Msg. Interval (s)']}$ & ${row['Avg. CPU Utilization (%)']}$ & ${row['Avg. Memory Utilization (KB)']}$ & ${row['Total CPU Energy (J)']}$ & ${row['CPU Power (W)']}$ \\\\\n"                    
            body += r"\cline{2-7}" + "\n"
        body += r"\hline" + "\n"

    footer = r"\end{tabular}"
    return header + body + footer

def dict_to_dataframe(dict, component):
    rows = []
    for entry in dict:
        for row in entry[component]:
            rows.append(row)
    columns = ['Language', '# Clients', 'Msg. Interval (s)', 'Avg. CPU Utilization (%)', 
            'Avg. Memory Utilization (KB)', 'Total CPU Energy (J)', 'CPU Power (W)']
    df = pd.DataFrame(rows, columns=columns)

    return df

# Factors
#algos = {'pubsub','service','action'}
#components = {'server', 'client'}
algos = {'action'}
components = {'server'}
intervals = {0.05,0.10,0.25,0.50,1.0}
intervals_list = list(sorted(intervals))
num_clients = {1,2,3}
num_clients_list = list(sorted(num_clients))
languages = {'py','cpp'}
languages_list = list(languages)

for algo in algos:
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
            num_rows=600
        case 'action':
            algo_folder='cpp_py_ros2_action'
            dest_folder='action'
            num_rows=600

    # Paths
    prefix = f"../exp_runners/experiments/{algo_folder}"
    s_folder = f"../exp_runners/experiments/{algo_folder}"
    root_d_folder = f"./tables/{dest_folder}"
    os.makedirs(root_d_folder, exist_ok=True)

    # Load Data Class
    l_data = LoadData(num_rows, s_folder, algo)
    l_run_table = l_data.load_run_table()

    data = []
    for component in components:
        component_power = []
        component_energy = []
        measurement_dict = {}
        component_cpu_df = l_data.load_cpu(component,False,False)
        component_mem_df = l_data.load_mem(component,False,False)
        component_energy_df = l_data.load_energy(component,False,False)
        component_power_df = l_data.load_power(component,False,False)
        for language in languages_list:
            for clients in num_clients_list:
                for interval in intervals_list:
                    print(f"lang: {language}, cli: {clients}, interval: {interval}")
                    # CPU
                    lang_component_cpu_df = l_data.filter_df(component_cpu_df,'language',language)
                    c_lang_component_cpu_df = l_data.filter_df(lang_component_cpu_df,'num_clients',clients)
                    i_c_lang_component_cpu_df = l_data.filter_df(c_lang_component_cpu_df,'msg_interval',interval)
                    cpu = str(round(i_c_lang_component_cpu_df["avg_cpu_pct"].mean(),2))

                    # Memory
                    lang_component_mem_df = l_data.filter_df(component_mem_df,'language',language)
                    c_lang_component_mem_df = l_data.filter_df(lang_component_mem_df,'num_clients',clients)
                    i_c_lang_component_mem_df = l_data.filter_df(c_lang_component_mem_df,'msg_interval',interval)
                    mem = str(round(i_c_lang_component_mem_df["avg_mem_pct"].mean(),2))

                    # Energy
                    lang_component_energy_df = l_data.filter_df(component_energy_df,'language',language)
                    c_lang_component_energy_df = l_data.filter_df(lang_component_energy_df,'num_clients',clients)
                    i_c_lang_component_energy_df = l_data.filter_df(c_lang_component_energy_df,'msg_interval',interval)
                    energy = str(round(i_c_lang_component_energy_df["sum_energy_pct"].mean(),2))

                    # Power
                    lang_component_power_df = l_data.filter_df(component_power_df,'language',language)
                    c_lang_component_power_df = l_data.filter_df(lang_component_power_df,'num_clients',clients)
                    i_c_lang_component_power_df = l_data.filter_df(c_lang_component_power_df,'msg_interval',interval)
                    power = str(round(i_c_lang_component_power_df["avg_energy_pct"].mean(),2))

                    # Dict
                    if component not in measurement_dict:
                        measurement_dict[component] = []
                    measurement_dict[component].append([language,str(clients),str(interval),cpu,mem,energy,power])

                continue
        
        data.append(measurement_dict)

        df = dict_to_dataframe(data,component)

        latex_table = generate_latex_table(df)

        print(latex_table)

        break