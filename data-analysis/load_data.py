import pandas as pd
import glob
import os
import numpy as np
from scipy.stats import boxcox

class LoadData:

    __num_rows = 0
    __s_folder = None
    __folder_01 = None
    __algo = None

    def __init__(self, num_rows: int, s_folder: str, algo: str):
        self.__num_rows = num_rows
        self.__s_folder = s_folder
        self.__folder_01 = s_folder+"_0_1"
        self.__algo = algo

    import pandas as pd

    def append_csv_files(self, run_table):
        other_table = pd.read_csv(f'{self.__folder_01}/run_table.csv')
        other_table['source'] = '2'
        appended_table = pd.concat([run_table, other_table], ignore_index=True)
        appended_table.to_csv('concatenated_file.csv', index=False)
        return appended_table

    def load_run_table(self):
        try:
            run_table = pd.read_csv(f'{self.__s_folder}/run_table.csv', nrows=self.__num_rows)
            run_table['source'] = '1'
            if self.__algo == 'pubsub':
                return self.append_csv_files(run_table)
            else:
                return run_table
        except:
            return None
        
    def log_transform(self, df, label):
        return df[label].apply(lambda x: np.log(x + 1) if x > 0 else np.nan)

    def boxcox_transform(self, df, label):
        return boxcox(df[label].dropna() + 1)
    
    def filter_df(self, df, label, value):
        return df[df[label] == value]
    
    def group_df_by(self, df, key, outliers):
        grouped_df = df.groupby(key)
        if not outliers:
            return grouped_df
        else:
            cleaned_df = grouped_df.apply(self.remove_outliers, column_name='avg_energy_pct')
            cleaned_df = cleaned_df.rename(columns={key: f'new_{key}'})
            #cleaned_df.reset_index()
            regrouped_df = cleaned_df.groupby(key)
            return regrouped_df
            
    def load_power(self, component, transform: bool, outliers: bool):
        df = self.load_run_table()
        runs_data = {}
        energy_files = {}
        avg_energy_df = {}
        #for run_id in df['__run_id']:
        for index, row in df.iterrows():
            run_id = row['__run_id']
            source = row['source']
            new_id = run_id+'_'+source
            df.loc[(df['__run_id'] == run_id) & (df['source'] == source), '__run_id'] = new_id
            if source == '1':
                folder_path = f"{self.__s_folder}/{run_id}"
            else:
                folder_path = f"{self.__folder_01}/{run_id}"
            energy_files = glob.glob(os.path.join(folder_path, f'energy-{component}-*.csv'))
            if energy_files:
                try:
                    energy_df = pd.read_csv(energy_files[0])
                    energy_df['CPU Power'] = pd.to_numeric(energy_df['CPU Power'], errors='coerce')
                    if transform:
                        # Apply log transformation (log + 1 to avoid log(0) issues)
                        # energy_df['CPU Power'] = self.log_transform(energy_df,'CPU Power')
                        # Apply Box-Cox transformation (ensure the values are positive)
                        energy_df['CPU Power'], _ = self.boxcox_transform(energy_df, 'CPU Power')
                    avg_energy_pct = energy_df['CPU Power'].mean()
                    runs_data[new_id] = avg_energy_pct
                except Exception as e:
                    print(f"Error processing file for run_id {run_id}: {e}")

        avg_energy_df = pd.DataFrame(list(runs_data.items()), columns=['__run_id', 'avg_energy_pct'])
        merged_df = df.merge(avg_energy_df, on='__run_id')

        clean_df = merged_df.fillna(0)

        return clean_df
    
    def load_energy(self, component, transform: bool, outliers: bool):
        df = self.load_run_table()
        runs_data = {}
        energy_files = {}
        sum_energy_df = {}
        #for run_id in df['__run_id']:
        for index, row in df.iterrows():
            run_id = row['__run_id']
            source = row['source']
            new_id = run_id+'_'+source
            df.loc[(df['__run_id'] == run_id) & (df['source'] == source), '__run_id'] = new_id
            if source == '1':
                folder_path = f"{self.__s_folder}/{run_id}"
            else:
                folder_path = f"{self.__folder_01}/{run_id}"
            energy_files = glob.glob(os.path.join(folder_path, f'energy-{component}-*.csv'))
            if energy_files:
                try:
                    energy_df = pd.read_csv(energy_files[0])
                    energy_df['CPU Power'] = pd.to_numeric(energy_df['CPU Power'], errors='coerce')
                    if transform:
                        # Apply log transformation (log + 1 to avoid log(0) issues)
                        # energy_df['CPU Power'] = self.log_transform(energy_df,'CPU Power')
                        # Apply Box-Cox transformation (ensure the values are positive)
                        energy_df['CPU Power'], _ = self.boxcox_transform(energy_df, 'CPU Power')
                    sum_energy_pct = energy_df['CPU Power'].sum()
                    runs_data[new_id] = sum_energy_pct
                except Exception as e:
                    print(f"Error processing file for run_id {run_id}: {e}")

        sum_energy_df = pd.DataFrame(list(runs_data.items()), columns=['__run_id', 'sum_energy_pct'])
        merged_df = df.merge(sum_energy_df, on='__run_id')
        
        clean_df = merged_df.fillna(0)

        return clean_df

    def load_cpu(self, component, transform: bool, outliers: bool):
        df = self.load_run_table()
        runs_data = {}
        energy_files = {}
        avg_cpu_df = {}
        #for run_id in df['__run_id']:
        for index, row in df.iterrows():
            run_id = row['__run_id']
            source = row['source']
            new_id = run_id+'_'+source
            df.loc[(df['__run_id'] == run_id) & (df['source'] == source), '__run_id'] = new_id
            if source == '1':
                folder_path = f"{self.__s_folder}/{run_id}"
            else:
                folder_path = f"{self.__folder_01}/{run_id}"
            energy_files = glob.glob(os.path.join(folder_path, f'energy-{component}-*.csv'))
            if energy_files:
                try:
                    cpu_df = pd.read_csv(energy_files[0])
                    cpu_df['CPU Power'] = pd.to_numeric(cpu_df['CPU Utilization'], errors='coerce')
                    if transform:
                        # Apply log transformation (log + 1 to avoid log(0) issues)
                        # energy_df['CPU Power'] = self.log_transform(energy_df,'CPU Power')
                        # Apply Box-Cox transformation (ensure the values are positive)
                        cpu_df['CPU Utilization'], _ = self.boxcox_transform(cpu_df, 'CPU Utilization')
                    avg_cpu_pct = cpu_df['CPU Utilization'].mean() * 100 * 8
                    runs_data[new_id] = avg_cpu_pct
                except Exception as e:
                    print(f"Error processing file for run_id {run_id}: {e}")

        avg_cpu_df = pd.DataFrame(list(runs_data.items()), columns=['__run_id', 'avg_cpu_pct'])
        merged_df = df.merge(avg_cpu_df, on='__run_id')

        clean_df = merged_df.fillna(0)

        return clean_df
    
    def load_mem(self, component, transform: bool, outliers: bool):
        df = self.load_run_table()
        runs_data = {}
        energy_files = {}
        avg_mem_df = {}
        for index, row in df.iterrows():
            run_id = row['__run_id']
            source = row['source']
            new_id = run_id+'_'+source
            df.loc[(df['__run_id'] == run_id) & (df['source'] == source), '__run_id'] = new_id
            if source == '1':
                folder_path = f"{self.__s_folder}/{run_id}"
            else:
                folder_path = f"{self.__folder_01}/{run_id}"
            energy_files = glob.glob(os.path.join(folder_path, f'cpu-mem-{component}.csv'))
            if energy_files:
                try:
                    mem_df = pd.read_csv(energy_files[0])
                    mem_df['memory_usage'] = pd.to_numeric(mem_df['memory_usage'], errors='coerce')
                    if transform:
                        mem_df['memory_usage'], _ = self.boxcox_transform(mem_df, 'memory_usage')
                    avg_mem_pct = mem_df['memory_usage'].mean() / 1024
                    runs_data[new_id] = avg_mem_pct
                except Exception as e:
                    print(f"Error processing file for run_id {run_id}: {e}")

        avg_mem_df = pd.DataFrame(list(runs_data.items()), columns=['__run_id', 'avg_mem_pct'])
        merged_df = df.merge(avg_mem_df, on='__run_id')

        clean_df = merged_df.fillna(0)

        return clean_df

    def remove_outliers(self, group, column_name):
        Q1 = group[column_name].quantile(0.25)
        Q3 = group[column_name].quantile(0.75)
        
        IQR = Q3 - Q1
        
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        return group[(group[column_name] >= lower_bound) & (group[column_name] <= upper_bound)]

    def load_all_energy():
        pass

    def load_energy_mean():
        pass

    def load_energy_total():
        pass

    def load_all_cpu():
        pass

    def load_all_memory():
        pass