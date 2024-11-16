import matplotlib.pyplot as plt
import pandas as pd
from collections import Counter

# Parse the log data
data = []
with open('LB8.txt', 'r') as file:
    for line in file:
        parts = line.split()
        if len(parts) >= 5:
            print(parts)
            timestamp = float(parts[4][1:-1])  # Extract timestamp
            command = parts[-1]  # Extract command
            data.append((timestamp, command))

# Convert to DataFrame
df = pd.DataFrame(data, columns=['timestamp', 'command'])
df['timestamp'] = df['timestamp'] - df['timestamp'].min()  # Normalize time

# Plot timeline of executed commands
plt.figure(figsize=(12, 6))
for command in df['command'].unique():
    command_data = df[df['command'] == command]
    plt.scatter(command_data['timestamp'], [command] * len(command_data), label=command, s=10)

plt.xlabel('Time (seconds)')
plt.ylabel('Command')
plt.title('Timeline of Executed Commands')
plt.legend()
plt.grid(True)
plt.savefig('command_timeline.png')
plt.close()

# Calculate command frequencies
command_counts = Counter(df['command'])
total_commands = sum(command_counts.values())
command_percentages = {cmd: count / total_commands * 100 for cmd, count in command_counts.items()}

# Plot histogram of command frequencies
plt.figure(figsize=(10, 6))
plt.bar(command_percentages.keys(), command_percentages.values())
plt.xlabel('Command')
plt.ylabel('Percentage of Time (%)')
plt.title('Histogram of Command Frequencies')
plt.savefig('command_histogram.png')
plt.close()

print("Graphs have been generated and saved as 'command_timeline.png' and 'command_histogram.png'.")