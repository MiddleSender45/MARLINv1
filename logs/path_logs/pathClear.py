import os
for f in os.listdir('/path_logs'):
    if f.endswith('.csv'):
        os.remove('/path_logs/' + f)
        print(f'Deleted {f}')