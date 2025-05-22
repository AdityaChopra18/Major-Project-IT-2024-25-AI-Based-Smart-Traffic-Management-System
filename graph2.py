import matplotlib.pyplot as plt
import numpy as np

# Labels and values
systems = ['Actuated Control(Static)', 'Our Smart System']
response_times = [74.2,10]
error = [2.5, 0]  # Error bar for Actuated (range: ±2.5), none for Smart

# Plot
plt.figure(figsize=(8, 5))
plt.bar(systems, response_times, yerr=error, capsize=10, color=['orange', 'green'])

# Style
plt.ylabel('Response Time (seconds)', fontname='Times New Roman')
plt.title('Response Time Analysis Graph', fontname='Times New Roman')
plt.xticks(fontname='Times New Roman')
plt.yticks(fontname='Times New Roman')
plt.grid(axis='y')
plt.tight_layout()
plt.savefig("response_time_analysis_graph.png", dpi=300)
plt.show()
