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
from pingouin import welch_anova as pg_welch_anova
from scipy import stats
import scikit_posthocs as sp
from statsmodels.formula.api import ols
from statsmodels.stats.multicomp import pairwise_tukeyhsd

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
        num_rows=320
    case 'action':
        pass

# Paths
prefix = f"../exp_runners/experiments/{algo_folder}"
s_folder = f"../exp_runners/experiments/{algo_folder}"
d_folder = f"./graphs/{dest_folder}"

# Ensure the output folder exists
os.makedirs(d_folder, exist_ok=True)

# Load Data Class
l_data = LoadData(num_rows, s_folder, algo)
l_run_table = l_data.load_run_table()

#####################
# Statistical Tests #
#####################

def shapiro_wilk(df, column_name, key):
    print("Starting Shapiro-Wilk test...")
    all_normal = True
    # if outliers:
    #     df = df.reset_index(drop=True)
    #     grouped = df.groupby(key)
    # else:
    #     grouped = df
    grouped = df

    num_groups = 0

    for group_name, group_data in grouped:
        data_to_test = group_data[column_name]
        
        if len(data_to_test) >= 3:
            stat, p_value = shapiro(data_to_test)
            
            print(f"Group: {group_name}")
            print(f"Shapiro-Wilk statistic: {stat}")
            print(f"P-value: {p_value}")
            
            if p_value > 0.05:
                print("The data is likely normally distributed (fail to reject H0).\n")
            else:
                all_normal = False
                print("The data is likely not normally distributed (reject H0).\n")
        else:
            print(f"Group {group_name} does not have enough data to run the Shapiro-Wilk test.\n")
        num_groups+=1
    if not all_normal:
        print("Shapiro-Wilk test indicates non-normal distribution among groups. Running Welch's ANOVA and Kruskal-Wallis...")
        welch_stat, welch_p = welch_anova(df, column_name)
        if num_groups >= 2:
            kruskal_wallis_test(df, column_name)
        else:
            print("Kruskal requires at least two groups.")
        print(f"Welch's ANOVA statistic: {welch_stat}, p-value: {welch_p}")
    return all_normal

def levene_test(grouped_df, value):
    group_values = [group[value].values for _, group in grouped_df]
    stat, p = levene(*group_values)
    print(f"Levene's test statistic: {stat}, p-value: {p}")
    if p < 0.05:
        print("Levene's test indicates unequal variances. Running Welch's ANOVA and Kruskal-Wallis...")
        welch_stat, welch_p = welch_anova(grouped_df, value)
        kruskal_wallis_test(grouped_df, value)
        print(f"Welch's ANOVA statistic: {welch_stat}, p-value: {welch_p}")
    else:
        # parametric
        print("Levene's test indicates equal variances. Running ANOVA...")
        one_way_anova(grouped_df)

def one_way_anova(grouped_df):
    groups = [group['avg_energy_pct'] for _, group in grouped_df]
    stat, p_value = f_oneway(*groups)
    print(f"ANOVA test: p = {p_value}")
    if p_value < 0.05:
         print(f"ANOVA test rejects the null hypothesis. There is statistical difference among groups.)")
         print("Since ANOVA is significant we perform Tukey's HSD test...")
         tukey_hsd(grouped_df)
    else:
        print(f"ANOVA test fails to reject the null hypothesis. There is not statistical difference among groups.)")

def welch_anova(grouped_df, value):
    group_values = [group[value].values for _, group in grouped_df]
    k = len(group_values)
    means = [group.mean() for group in group_values]
    variances = [group.var(ddof=1) for group in group_values]
    ns = [len(group) for group in group_values]
    print(f"Group means: {means}")
    print(f"Group variances: {variances}")
    print(f"Group sizes: {ns}")
    numerator = sum((mean - sum(means)/k) ** 2 / var for mean, var in zip(means, variances))
    denominator = sum(var / n for var, n in zip(variances, ns))
    print(f"Numerator: {numerator}")
    print(f"Denominator: {denominator}")
    welch_stat = numerator / denominator
    df = sum(1 / (n - 1) for n in ns)
    print(f"Degrees of freedom: {df}")
    welch_p = 1 - stats.chi2.cdf(welch_stat, df)
    return welch_stat, welch_p

def kruskal_wallis_test(grouped_df, value):
    group_values = [group[value].values for _, group in grouped_df]
    stat, p = stats.kruskal(*group_values)
    print(f"Kruskal-Wallis H statistic: {stat}, p-value: {p}")
    
    if p < 0.05:
        print("The test indicates a significant difference between the groups. Running post-hoc Dunn's test")
        dunn_test(grouped_df, value)
    else:
        print("The test indicates no significant difference between the groups.")

def dunn_test(grouped_df, value):
    group_values = [group[value].values for _, group in grouped_df]
    all_values = []
    group_labels = []
    for label, group in grouped_df:
        all_values.extend(group[value].values)
        group_labels.extend([label] * len(group))
    data = pd.DataFrame({
        'values': all_values,
        'group': group_labels
    })
    dunn_result = sp.posthoc_dunn(data, val_col='values', group_col='group', p_adjust='bonferroni')
    print("Dunn's test results (adjusted p-values):")
    print(dunn_result)
    return dunn_result

def tukey_hsd(grouped_df):
    values = []
    labels = []
    for group_name, group in grouped_df:
        values.extend(group['avg_energy_pct'])
        labels.extend([group_name] * len(group))
    tukey_result = pairwise_tukeyhsd(values, labels, alpha=0.05)
    print("\nTukey's HSD Test Results:")
    print(tukey_result)

def gen_single_density_graph(df, filter: list):
    for (keys), group in df:
        sns.histplot(
            group['avg_energy_pct'], 
            bins=10, 
            kde=True, 
            stat="density",
            label=f"{keys}", 
            alpha=0.5
        )
    plt.xlabel("Power (W)")
    plt.ylabel("Density")
    plt.legend()

    variables = [algo, component, "density_energy", "_".join(filter)]

    # PNG
    output_file_name = "_".join(str(var) for var in variables) + ".png"
    output_path = os.path.join(d_folder, output_file_name)
    plt.savefig(output_path, format="png", dpi=300)
    # PDF
    output_file_name = "_".join(str(var) for var in variables) + ".pdf"
    output_path = os.path.join(d_folder, output_file_name)
    plt.savefig(output_path, format="pdf", dpi=300)
    #plt.show()
    plt.close()

def gen_boxplot_graph(df, filter):
    plt.figure(figsize=(3, 3))
    sns.boxplot(data=df, x='language', y='avg_energy_pct', hue='num_clients')
    plt.xlabel("Language")
    plt.ylabel("Power (W)")
    plt.tight_layout()

    variables = [algo, component, "energy_boxplot", "_".join(filter)]

    # PNG
    output_file_name = "_".join(str(var) for var in variables) + ".png"
    output_path = os.path.join(d_folder, output_file_name)
    plt.savefig(output_path, format="png", dpi=300)
    # PDF
    output_file_name = "_".join(str(var) for var in variables) + ".pdf"
    output_path = os.path.join(d_folder, output_file_name)
    plt.savefig(output_path, format="pdf", dpi=300)
    plt.close()

# Factors
#components = {'server', 'client'}
components = {'server'}
intervals = {0.05,0.1,0.25,0.5,1.0}
num_clients = {1,2,3}
languages = {'py','cpp'}

# Data Cleaning
transformation = True
outliers = True

### FOR energy / CPU / ETC
for component in components:
    df_load = l_data.load_power(component, transformation, outliers)
    print(f"Component: {component} --->")
    grouped_df = l_data.group_df_by(df_load,'language',outliers)
    # if outliers:
    #     grouped_df = grouped_df.group_by('language')
    if shapiro_wilk(grouped_df, 'avg_energy_pct', 'language'):
        print("Levene's test for different languages.")
        levene_test(grouped_df,'avg_energy_pct')
        gen_single_density_graph(grouped_df,{'py_and_cpp'})
    for language in languages:
        df = df_load
        df_language = l_data.filter_df(df,'language',language)
        print(f"Component: {component}, Language: {language} --->")
        cleaned_df = l_data.group_df_by(df_language,'language',outliers)
        for group_name, group_data in cleaned_df:
            print(f"Group: {group_name}")
            print(group_data['avg_energy_pct'].to_numpy())
            gen_single_density_graph(cleaned_df, {str(language)})
        shapiro_wilk(cleaned_df, 'avg_energy_pct', 'language')
        grouped_df = l_data.group_df_by(df_language,'msg_interval',outliers)
        if shapiro_wilk(grouped_df, 'avg_energy_pct', 'msg_interval'):
            print(f"Levene's test for language {language} and different msg_intervals.")
            levene_test(grouped_df,'avg_energy_pct')
            gen_single_density_graph(grouped_df,{language,'intervals'})
        for interval in intervals:
            df = df_language
            df = l_data.filter_df(df,'msg_interval',interval)
            df_interval = df
            print(f"Component: {component}, Language: {language}, Interval: {interval},  --->")
            cleaned_df = l_data.group_df_by(df,'msg_interval', outliers)
            for group_name, group_data in cleaned_df:
                print(f"Group: {group_name}")
                print(group_data['avg_energy_pct'].to_numpy())
                gen_single_density_graph(cleaned_df, {str(language),str(interval)})
            shapiro_wilk(cleaned_df, 'avg_energy_pct', 'msg_interval')
            # Levene's
            # levene_test(cleaned_df,'avg_energy_pct')
            # ANOVA (one way)
            # groups = [group['avg_energy_pct'] for _, group in grouped_df]
            # stat, p_value = f_oneway(*groups)
            # print(f"ANOVA test: p = {p_value}")
            grouped_df = l_data.group_df_by(df_interval,'num_clients',outliers)
            if shapiro_wilk(grouped_df,'avg_energy_pct','num_clients'):
                print(f"Levene's test for language {language}, msg_interval {interval} and different num_clients.")
                levene_test(grouped_df,'avg_energy_pct')
                gen_single_density_graph(grouped_df,{str(language),str(interval),'clients'})
            for clients in num_clients: # Only for graph and normal distribution assertion
                df = df_interval
                df = l_data.filter_df(df,'num_clients',clients)
                print(f"Component: {component}, Language: {language}, Interval: {interval}, Clients: {clients} --->")
                cleaned_df = l_data.group_df_by(df,'num_clients', outliers)
                for group_name, group_data in cleaned_df:
                    print(f"Group: {group_name}")
                    print(group_data['avg_energy_pct'].to_numpy())
                    gen_single_density_graph(cleaned_df, {str(language),str(interval), str(clients)})
                shapiro_wilk(cleaned_df, 'avg_energy_pct', 'num_clients')
                # grouped_df = l_data.group_df_by(df_language,'num_clients',outliers)
                # if shapiro_wilk(grouped_df, 'avg_energy_pct','num_clients'):
                #     print(f"Levene's test for language {language} and different num_clients.")
                #     levene_test(grouped_df,'avg_energy_pct')
                #     gen_single_density_graph(grouped_df,{language,'clients'})
        grouped_df = l_data.group_df_by(df_language,'num_clients',outliers)
        if shapiro_wilk(grouped_df, 'avg_energy_pct', 'num_clients'):
            print(f"Levene's test for language {language} and different num_clients.")
            levene_test(grouped_df,'avg_energy_pct')
            gen_single_density_graph(grouped_df,{language,'clients'})
        for clients in num_clients:
                df = df_language
                df = l_data.filter_df(df,'num_clients',clients)
                print(f"Component: {component}, Clients: {clients}, Language: {language} --->")
                cleaned_df = l_data.group_df_by(df,'num_clients', outliers)
                for group_name, group_data in cleaned_df:
                    print(f"Group: {group_name}")
                    print(group_data['avg_energy_pct'].to_numpy())
                    gen_single_density_graph(cleaned_df, {str(language), str(clients)})
                shapiro_wilk(cleaned_df, 'avg_energy_pct', 'num_clients')
                # 1 client and different msg intervals
                if clients == 1:
                    grouped_df = l_data.group_df_by(df,'msg_interval',outliers)
                    if shapiro_wilk(grouped_df,'avg_energy_pct','msg_interval'):
                        print(f"Levene's test for language {language}, num_clients {clients} and different msg_intervals.")
                        levene_test(grouped_df,'avg_energy_pct')
                        gen_single_density_graph(grouped_df,{str(language),str(clients),'interval'})
                    #shapiro_wilk(grouped_df, 'avg_energy_pct', 'msg_interval')
    if shapiro_wilk(grouped_df, 'avg_energy_pct', 'num_clients'):
        print(f"Levene's test different num_clients.")
        grouped_df = l_data.group_df_by(df_load,'num_clients',outliers)
        levene_test(grouped_df,'avg_energy_pct')
        gen_single_density_graph(grouped_df,{'clients'})
    for clients in num_clients:
        df = df_load
        df_clients = l_data.filter_df(df,'num_clients',clients)
        print(f"Component: {component}, Num clients: {clients} --->")
        cleaned_df = l_data.group_df_by(df_clients,'num_clients',outliers)
        for group_name, group_data in cleaned_df:
            print(f"Group: {group_name}")
            print(group_data['avg_energy_pct'].to_numpy())
            gen_single_density_graph(cleaned_df, {str(num_clients)})
        shapiro_wilk(cleaned_df, 'avg_energy_pct', 'num_clients')

    grouped_df = l_data.group_df_by(df_load,'num_clients',outliers)
    if shapiro_wilk(grouped_df,'avg_energy_pct','msg_interval'):
        print(f"Levene's different msg_intervals.")
        levene_test(grouped_df,'avg_energy_pct')
        gen_single_density_graph(grouped_df,{'intervals'})
    for interval in intervals:
        df = df_load
        df_intervals = l_data.filter_df(df,'msg_interval',interval)
        print(f"Component: {component}, Interval: {interval} --->")
        cleaned_df = l_data.group_df_by(df_intervals,'msg_interval',outliers)
        for group_name, group_data in cleaned_df:
            print(f"Group: {group_name}")
            print(group_data['avg_energy_pct'].to_numpy())
            gen_single_density_graph(cleaned_df, {str(interval)})
        shapiro_wilk(cleaned_df, 'avg_energy_pct', 'msg_interval')
        gen_boxplot_graph(df_intervals,{str(interval)})