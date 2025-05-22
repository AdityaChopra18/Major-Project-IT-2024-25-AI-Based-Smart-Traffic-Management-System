import matplotlib.pyplot as plt
import numpy as np

# Categories
categories = ['Adaptability', 'Scalability', 'Implementation Cost', 'Maintenance Requirements']

# Map qualitative features to numerical values
traditional = [0, 3, 1, 1]         # None, High, Low, Low
actuated = [1, 2, 2, 2]           # Limited, Medium, Medium, Medium
smart = [3, 3, 3, 2.5]            # High, High, High, Medium-High

x = np.arange(len(categories))
width = 0.25

# Plot
plt.figure(figsize=(10, 6))
plt.bar(x - width, traditional, width, label='Traditional Fixed-Time')
plt.bar(x, actuated, width, label='Actuated Control')
plt.bar(x + width, smart, width, label='Our Smart System')

# Labels and formatting
plt.ylabel('Qualitative Score (0–3)', fontname='Times New Roman')
plt.title('Performance Comparison Chart', fontname='Times New Roman')
plt.xticks(x, categories, rotation=20, fontname='Times New Roman')
plt.yticks([0, 1, 2, 2.5, 3], ['None', 'Limited/Low', 'Medium', 'Med-High', 'High'], fontname='Times New Roman')
plt.legend()
plt.grid(axis='y')
plt.tight_layout()
plt.savefig("performance_comparison_qualitative.png", dpi=300)
plt.show()
