import os
for f in os.listdir('/telemetry_logs'):
    if f.endswith('.csv'):
        os.remove('/telemetry_logs/' + f)
        print(f'Deleted {f}')